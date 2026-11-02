package cloud

// The encoder tests pin bytes that were captured from the vendor, not bytes
// this program invented. Where a command was confirmed on hardware the hex is
// quoted from the document that recorded it, so a refactor that changes the
// wire format fails here rather than on the robot.

import (
	"encoding/json"
	"github.com/dhumpf/x10-ctl/internal/dp"
	"github.com/dhumpf/x10-ctl/internal/mqtt"
	"github.com/dhumpf/x10-ctl/internal/pb"
	"strings"
	"testing"
	"time"
)

// The envelope's two non-obvious properties: payload is a serialised string,
// and our client_id is the literal "cloud" so our own echoes can be dropped.
func TestEnvelopeShape(t *testing.T) {
	c := NewCloud("AMP00X0E00000000", "eufy_home")
	body, err := c.envelope(cloudPayload{
		Protocol: 2, T: 1, DeviceSN: "AMP00X0E00000000",
		Data: map[string]any{"160": true},
	})
	if err != nil {
		t.Fatal(err)
	}
	var outer struct {
		Head    cloudHead       `json:"head"`
		Payload json.RawMessage `json:"payload"`
	}
	if err := json.Unmarshal(body, &outer); err != nil {
		t.Fatal(err)
	}
	if outer.Payload[0] != '"' {
		t.Fatalf("payload must be a JSON string, got %s", outer.Payload[:1])
	}
	if outer.Head.ClientID != cloudClientID {
		t.Errorf("client_id = %q, want %q", outer.Head.ClientID, cloudClientID)
	}
	if outer.Head.Cmd != 65537 || outer.Head.CmdStatus != 2 {
		t.Errorf("head cmd/cmd_status = %d/%d, want 65537/2", outer.Head.Cmd, outer.Head.CmdStatus)
	}
	p, err := decodePayload(outer.Payload)
	if err != nil {
		t.Fatal(err)
	}
	// A Bool DP goes as a bare JSON true. Every base64 form was ignored.
	if v, ok := p.Data["160"].(bool); !ok || !v {
		t.Errorf("dp 160 = %#v, want bool true", p.Data["160"])
	}
}

// The robot nests payload; the cloud and the app stringify it. Parsing has to
// take both or every device report is discarded.
func TestDecodePayloadBothDirections(t *testing.T) {
	nested := `{"protocol":1,"t":5,"data":{"163":88}}`
	p, err := decodePayload(json.RawMessage(nested))
	if err != nil || p.Protocol != 1 {
		t.Fatalf("nested: %v, protocol %d", err, p.Protocol)
	}
	quoted, _ := json.Marshal(nested)
	p, err = decodePayload(quoted)
	if err != nil || p.Protocol != 1 {
		t.Fatalf("stringified: %v, protocol %d", err, p.Protocol)
	}
}

// Observed live 2026-07-30: with the robot off its dock, work_status arrives
// carrying fields 14 and 15 and no field 2. That is a partial update, not a
// transition to an unknown state, and it must not clobber the known one.
func TestPartialWorkStatusKeepsLastState(t *testing.T) {
	cloud := NewCloud("AMP00X0E00000000", "eufy_home")
	full := dp.DPValue(pb.FieldVarint(2, 5))
	cloud.applyReport(map[string]any{"153": full})
	if got := cloud.State().Status; got != "cleaning" {
		t.Fatalf("state = %q, want cleaning", got)
	}

	partial := dp.DPValue(append(pb.FieldBytes(14, []byte{1, 2}), pb.FieldBytes(15, nil)...))
	if _, ok := dp.WorkStateOf(partial); ok {
		t.Fatal("a report with no field 2 must not yield a state")
	}
	cloud.applyReport(map[string]any{"153": partial})
	st := cloud.State()
	if st.Status != "cleaning" {
		t.Errorf("state after partial report = %q, want cleaning held", st.Status)
	}
	// the raw view still shows what actually arrived
	if st.DataPoint["work_status"] != "14:2B 15:0B" {
		t.Errorf("data_points work_status = %q", st.DataPoint["work_status"])
	}
}

// --- MQTT wire

// --- end to end, both halves of the default mode

// A stand-in robot connects to the embedded broker, reports its battery, and
// receives a beep. Plain TCP: the TLS the real robot insists on is the one
// thing that cannot be exercised without its trust store.
func TestBrokerAndCloudEndToEnd(t *testing.T) {
	const sn = "AMP00X0E00000000"
	cloud := NewCloud(sn, "eufy_home")
	broker, err := mqtt.NewBrokerBus("127.0.0.1:0", "", "", true, cloud.Handle)
	if err != nil {
		t.Fatal(err)
	}
	cloud.SetBus(broker)

	stop := make(chan struct{})
	defer close(stop)
	go broker.Run(stop)

	addr := waitFor(t, "broker to bind", func() (string, bool) {
		a := broker.Addr()
		return a, !strings.HasSuffix(a, ":0")
	})

	inbox := make(chan e2eMsg, 16)
	robot := mqtt.NewClientBus(addr, "robot", "", false, true, func(topic string, payload []byte) {
		inbox <- e2eMsg{Topic: topic, Payload: payload}
	})
	go robot.Run(stop)
	waitFor(t, "robot to connect", func() (string, bool) {
		return "", robot.Status().Connected
	})

	// The robot reports 88% on the topic it chose, which is where the cloud has
	// to learn to reply - the app-name segment is not assumed.
	res := "cmd/eufy_home/T2351/" + sn + "/res"
	report := `{"head":{"client_id":"robot"},"payload":{"protocol":1,"t":1,"device_sn":"` + sn +
		`","account_id":"acct","data":{"163":88,"153":"` + dp.DPValue(pb.FieldVarint(2, 3)) + `"}}}`
	if err := robot.Publish(res, []byte(report)); err != nil {
		t.Fatal(err)
	}
	waitFor(t, "battery report", func() (string, bool) {
		return "", cloud.State().Battery == 88
	})
	if st := cloud.State(); st.Status != "docked" || st.ReqTopic != "cmd/eufy_home/T2351/"+sn+"/req" {
		t.Errorf("state after report: %+v", st)
	}

	if err := cloud.Beep(true); err != nil {
		t.Fatal(err)
	}
	deadline := time.After(5 * time.Second)
	for {
		select {
		case msg := <-inbox:
			p, err := payloadOf(msg.Payload)
			if err != nil || p.Protocol != 2 {
				continue // the protocol 4 online notice arrives first
			}
			if v, ok := p.Data["160"].(bool); !ok || !v {
				t.Fatalf("beep carried %#v, want bool true", p.Data["160"])
			}
			if msg.Topic != "cmd/eufy_home/T2351/"+sn+"/req" {
				t.Fatalf("beep went to %q", msg.Topic)
			}
			return
		case <-deadline:
			t.Fatal("no protocol 2 command reached the robot")
		}
	}
}

// Publishing with nothing connected has to say so plainly - the usual cause is
// the robot still configured to reach a different broker.
func TestPublishWithNoClientExplainsItself(t *testing.T) {
	cloud := NewCloud("AMP00X0E00000000", "eufy_home")
	broker, err := mqtt.NewBrokerBus("127.0.0.1:0", "", "", true, cloud.Handle)
	if err != nil {
		t.Fatal(err)
	}
	cloud.SetBus(broker)
	err = cloud.Beep(true)
	if err == nil || !strings.Contains(err.Error(), "configured to use this broker") {
		t.Errorf("error was %v, want a hint about the robot's broker address", err)
	}
}

func TestMQTTDisabledIsNotASilentNoOp(t *testing.T) {
	cloud := NewCloud("AMP00X0E00000000", "eufy_home")
	if err := cloud.Beep(true); err == nil {
		t.Error("beep with no bus returned nil; it must not look like it worked")
	}
}

func payloadOf(b []byte) (cloudPayload, error) {
	var msg struct {
		Payload json.RawMessage `json:"payload"`
	}
	if err := json.Unmarshal(b, &msg); err != nil {
		return cloudPayload{}, err
	}
	return decodePayload(msg.Payload)
}

func waitFor(t *testing.T, what string, cond func() (string, bool)) string {
	t.Helper()
	deadline := time.Now().Add(5 * time.Second)
	for time.Now().Before(deadline) {
		if v, ok := cond(); ok {
			return v
		}
		time.Sleep(10 * time.Millisecond)
	}
	t.Fatalf("timed out waiting for %s", what)
	return ""
}

type e2eMsg struct {
	Topic   string
	Payload []byte
}
