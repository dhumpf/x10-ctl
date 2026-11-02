package mqtt

// The encoder tests pin bytes that were captured from the vendor, not bytes
// this program invented. Where a command was confirmed on hardware the hex is
// quoted from the document that recorded it, so a refactor that changes the
// wire format fails here rather than on the robot.

import (
	"bufio"
	"bytes"
	"testing"
)

func newBufReader(b []byte) *bufio.Reader { return bufio.NewReader(bytes.NewReader(b)) }

// --- MQTT wire

func TestPacketRoundTrip(t *testing.T) {
	pub := makePublish("cmd/eufy_home/T2351/AMP00X0E00000000/req", []byte(`{"a":1}`), 7, 1, false)
	got := mustReadPacket(t, pub.encode())
	msg, err := parsePublish(got)
	if err != nil {
		t.Fatal(err)
	}
	if msg.Topic != "cmd/eufy_home/T2351/AMP00X0E00000000/req" || string(msg.Payload) != `{"a":1}` || msg.ID != 7 {
		t.Errorf("publish round trip: %+v", msg)
	}

	conn := mustReadPacket(t, makeConnect("android-eufy_home-abc-def", 60, connectOpts{}).encode())
	id, keepalive, err := parseConnect(conn)
	if err != nil || id != "android-eufy_home-abc-def" || keepalive != 60 {
		t.Errorf("connect round trip: %q %d %v", id, keepalive, err)
	}

	sub := mustReadPacket(t, makeSubscribe(3, "#", "cmd/+/T2351/+/res").encode())
	pid, filters, err := parseSubscribe(sub)
	if err != nil || pid != 3 || len(filters) != 2 || filters[1] != "cmd/+/T2351/+/res" {
		t.Errorf("subscribe round trip: %d %v %v", pid, filters, err)
	}
}

// A remaining length over 127 needs a second varint byte; a payload that size
// is routine on the bulk channel.
func TestPacketLargeRemainingLength(t *testing.T) {
	payload := make([]byte, 5000)
	for i := range payload {
		payload[i] = byte(i)
	}
	got := mustReadPacket(t, makePublish("t", payload, 0, 0, false).encode())
	msg, err := parsePublish(got)
	if err != nil {
		t.Fatal(err)
	}
	if len(msg.Payload) != 5000 || msg.Payload[4999] != payload[4999] {
		t.Errorf("large payload mangled: %d bytes", len(msg.Payload))
	}
}

func TestTopicMatches(t *testing.T) {
	for _, tc := range []struct {
		filter, topic string
		want          bool
	}{
		{"#", "cmd/eufy_home/T2351/SN/res", true},
		{"cmd/+/T2351/SN/res", "cmd/eufy_home/T2351/SN/res", true},
		{"cmd/+/T2351/SN/res", "cmd//T2351/SN/res", true}, // app name lost after a reset
		{"cmd/eufy_home/T2351/SN/req", "cmd/eufy_home/T2351/SN/res", false},
		{"cmd/eufy_home/#", "cmd/eufy_home/T2351/SN/res", true},
		{"cmd/+/res", "cmd/eufy_home/T2351/SN/res", false},
	} {
		if got := topicMatches(tc.filter, tc.topic); got != tc.want {
			t.Errorf("topicMatches(%q,%q) = %v, want %v", tc.filter, tc.topic, got, tc.want)
		}
	}
}

func mustReadPacket(t *testing.T, b []byte) packet {
	t.Helper()
	p, err := readPacket(newBufReader(b))
	if err != nil {
		t.Fatalf("readPacket: %v", err)
	}
	return p
}

// --- end to end, both halves of the default mode

// The cloud session must keep sending exactly the CONNECT it always sent. The
// will and credential fields were added for Home Assistant, and a zero
// connectOpts has to leave the robot-facing bytes untouched - flags 0x02, and
// a payload that ends at the client id.
func TestZeroConnectOptsIsByteIdentical(t *testing.T) {
	got := makeConnect("android-eufy_home-abc-def", 60, connectOpts{})
	want := []byte{0, 4, 'M', 'Q', 'T', 'T', 4, 0x02, 0, 60}
	want = append(want, mqttString("android-eufy_home-abc-def")...)
	if !bytes.Equal(got.body, want) {
		t.Errorf("CONNECT body changed\n got %x\nwant %x", got.body, want)
	}
	if got.flags != 0 {
		t.Errorf("CONNECT fixed-header flags = %#x, want 0", got.flags)
	}
}

// MQTT 3.1.1 3.1.2.3 and 3.1.3: the flag bits and the payload order. A broker
// given these in the wrong order closes the socket without an error code, so
// the bytes are pinned rather than the behaviour.
func TestConnectWithWillAndCredentials(t *testing.T) {
	got := makeConnect("x10-ctl-ha", 60, connectOpts{
		Username: "ha",
		Password: "pw",
		Will:     &Will{Topic: "x10/avail", Payload: []byte("offline"), QoS: 1, Retain: true},
	})

	// 0x02 clean session | 0x04 will | 0x08 will QoS 1 | 0x20 will retain
	// | 0x40 password | 0x80 username
	const wantFlags = 0x02 | 0x04 | 0x08 | 0x20 | 0x40 | 0x80
	if got.body[7] != wantFlags {
		t.Errorf("connect flags = %#x, want %#x", got.body[7], wantFlags)
	}

	want := []byte{0, 4, 'M', 'Q', 'T', 'T', 4, wantFlags, 0, 60}
	want = append(want, mqttString("x10-ctl-ha")...)
	want = append(want, mqttString("x10/avail")...)
	want = append(want, mqttBytes([]byte("offline"))...)
	want = append(want, mqttString("ha")...)
	want = append(want, mqttString("pw")...)
	if !bytes.Equal(got.body, want) {
		t.Errorf("CONNECT body\n got %x\nwant %x", got.body, want)
	}

	// The parser only ever reads the client id, and must not be confused by
	// the fields that now follow it.
	id, keepalive, err := parseConnect(got)
	if err != nil || id != "x10-ctl-ha" || keepalive != 60 {
		t.Errorf("parseConnect after will: %q %d %v", id, keepalive, err)
	}
}

// Retain lives in bit 0 of the fixed-header nibble, next to QoS in bits 1-2.
// Discovery configs are retained and commands must not be, so both directions
// are asserted.
func TestPublishRetainFlag(t *testing.T) {
	for _, tc := range []struct {
		qos    byte
		retain bool
		want   byte
	}{
		{0, false, 0x00},
		{0, true, 0x01},
		{1, false, 0x02},
		{1, true, 0x03},
	} {
		got := makePublish("t", []byte("x"), 1, tc.qos, tc.retain)
		if got.flags != tc.want {
			t.Errorf("qos=%d retain=%v: flags = %#x, want %#x",
				tc.qos, tc.retain, got.flags, tc.want)
		}
		// The flag must survive a trip through the encoder and reader.
		back := mustReadPacket(t, got.encode())
		if back.flags&0x01 != 0 != tc.retain {
			t.Errorf("qos=%d retain=%v: lost in round trip (flags %#x)",
				tc.qos, tc.retain, back.flags)
		}
	}
}

// MQTT-2.3.1-1 forbids packet id 0. The old uint16(nextID.Add(1)) emitted one
// on every 65536th publish, which mosquitto answers by closing the connection.
func TestPacketIDNeverZero(t *testing.T) {
	c := &ClientBus{}
	c.nextID.Store(65534)
	for i := 0; i < 4; i++ {
		if id := c.packetID(); id == 0 {
			t.Fatalf("publish %d got packet id 0", i)
		}
	}
	// And the wrap actually happened, so the test is not passing vacuously.
	if c.nextID.Load() < 65536 {
		t.Errorf("counter did not wrap: %d", c.nextID.Load())
	}
}
