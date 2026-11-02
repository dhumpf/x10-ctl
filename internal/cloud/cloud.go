package cloud

// The cloud half of the conversation.
//
// x10-ctl plays Anker's cloud: it answers what the robot asks, folds the
// robot's reports into state, and pushes data-point commands back. That role is
// the same whether the robot reached us through the embedded broker or through
// a broker on another host, so nothing here knows which Bus it has.
//
// Every shape below came from the 2026-07-28 capture of the real cloud, not
// from the reference clients, and two of them could not have been guessed:
//
//   - a cloud_time answer is protocol 8 with res_type/res_value. 78 swept
//     shapes missed it.
//   - `payload` is a serialised JSON STRING, not a nested object, in the
//     cloud-to-device direction. The robot ignores the nested form.
//
// The robot reports with `payload` as a nested object, so parsing accepts both.

import (
	"crypto/rand"
	"encoding/hex"
	"encoding/json"
	"fmt"
	"github.com/dhumpf/x10-ctl/internal/dp"
	"github.com/dhumpf/x10-ctl/internal/mqtt"
	"github.com/dhumpf/x10-ctl/internal/pb"
	"log"
	"strconv"
	"strings"
	"sync"
	"sync/atomic"
	"time"
)

const modelCode = "T2351"

type cloudHead struct {
	Version   string `json:"version"`
	ClientID  string `json:"client_id"`
	SessID    string `json:"sess_id"`
	Cmd       int    `json:"cmd"`
	CmdStatus int    `json:"cmd_status"`
	MsgSeq    uint64 `json:"msg_seq"`
	SignCode  int    `json:"sign_code"`
	Seed      string `json:"seed"`
	Timestamp int64  `json:"timestamp"`
}

// cloudClientID is what we stamp on everything we send. Inbound messages
// carrying it are our own, echoed back by the broker, and are dropped.
const cloudClientID = "cloud"

type cloudPayload struct {
	Protocol  int            `json:"protocol"`
	T         int64          `json:"t"`
	DeviceSN  string         `json:"device_sn,omitempty"`
	AccountID string         `json:"account_id,omitempty"`
	Data      map[string]any `json:"data,omitempty"`
}

type Cloud struct {
	bus  mqtt.Bus
	seq  atomic.Uint64 // envelope head.msg_seq
	sess string
	// cmdSeq is the ModeCtrlRequest sequence, a separate counter that climbs
	// across commands the way the app's does. Seeded above the low numbers so
	// a fresh start does not collide with something already in flight.
	cmdSeq     atomic.Uint64
	appDefault string

	mu        sync.RWMutex
	sn        string
	accountID string
	reqTopic  string
	resTopic  string
	dps       map[int]string // decoded, for display
	battery   int
	workState string // last report that actually carried one; see workStateOf
	lastSeen  time.Time
	lastErr   string
	announced map[string]bool
}

// CloudState is the JSON the UI reads.
type CloudState struct {
	Bus       mqtt.BusStatus    `json:"bus"`
	ReqTopic  string            `json:"req_topic"`
	ResTopic  string            `json:"res_topic"`
	Battery   int               `json:"battery"`
	Status    string            `json:"work_status"`
	DataPoint map[string]string `json:"data_points"`
	// a pointer, because omitempty does not drop a zero time.Time and
	// "0001-01-01" reads as data rather than as "never heard from"
	LastSeen  *time.Time `json:"last_seen,omitempty"`
	LastError string     `json:"last_error,omitempty"`
}

func NewCloud(sn, appName string) *Cloud {
	b := make([]byte, 16)
	_, _ = rand.Read(b)
	c := &Cloud{
		sess:       hex.EncodeToString(b),
		appDefault: appName,
		sn:         sn,
		dps:        map[int]string{},
		battery:    -1,
		announced:  map[string]bool{},
	}
	c.cmdSeq.Store(100)
	c.setTopics(appName, sn)
	return c
}

func (c *Cloud) SetBus(b mqtt.Bus) { c.bus = b }

// HasBus reports whether a data-point path was configured at startup. The HA
// bridge lends its own connection only when this is false, so a live internal
// broker is never replaced during the window before a client has dialled in.
func (c *Cloud) HasBus() bool { return c != nil && c.bus != nil }

// Ready reports whether a data point could plausibly be delivered right now.
// Used to decide whether falling back from the local path is worth trying.
func (c *Cloud) Ready() bool {
	return c != nil && c.bus != nil && c.bus.Status().Connected
}

// SetIdentity refreshes the serial once discovery learns it, so MQTT works with
// the same zero-configuration promise the 9668 path has.
func (c *Cloud) SetIdentity(sn string) {
	if sn == "" {
		return
	}
	c.mu.Lock()
	changed := c.sn != sn
	c.sn = sn
	if changed {
		c.setTopics(c.appDefault, sn)
	}
	topic := c.reqTopic
	c.mu.Unlock()
	if changed {
		log.Printf("cloud: topics now %s", topic)
	}
}

// setTopics must be called with the lock held, or before the Cloud is shared.
func (c *Cloud) setTopics(app, sn string) {
	if sn == "" {
		return
	}
	base := fmt.Sprintf("cmd/%s/%s/%s", app, modelCode, sn)
	c.reqTopic, c.resTopic = base+"/req", base+"/res"
}

func (c *Cloud) State() CloudState {
	c.mu.RLock()
	defer c.mu.RUnlock()
	st := CloudState{
		ReqTopic: c.reqTopic, ResTopic: c.resTopic, Battery: c.battery,
		DataPoint: map[string]string{}, LastError: c.lastErr,
	}
	if !c.lastSeen.IsZero() {
		seen := c.lastSeen
		st.LastSeen = &seen
	}
	for _, id := range dp.SortedDPs(c.dps) {
		st.DataPoint[dp.DPName(id)] = c.dps[id]
	}
	st.Status = c.workState
	if c.bus != nil {
		st.Bus = c.bus.Status()
	}
	return st
}

// --- outbound

func (c *Cloud) head() cloudHead {
	return cloudHead{
		Version: "1.0.0.1", ClientID: cloudClientID, SessID: c.sess,
		Cmd: 65537, CmdStatus: 2, MsgSeq: c.seq.Add(1), SignCode: 0,
		Seed: "", Timestamp: time.Now().Unix(),
	}
}

func (c *Cloud) envelope(p cloudPayload) ([]byte, error) {
	inner, err := json.Marshal(p)
	if err != nil {
		return nil, err
	}
	// payload is a string here on purpose - see the file comment.
	return json.Marshal(struct {
		Head    cloudHead `json:"head"`
		Payload string    `json:"payload"`
	}{c.head(), string(inner)})
}

// SendDP publishes a protocol 2 command carrying one or more data points.
//
// Values must already be in the form the DP's declared type wants: a base64
// string for Raw, a bare Go bool/int/string for everything else.
func (c *Cloud) SendDP(data map[string]any) error {
	if c.bus == nil {
		return fmt.Errorf("MQTT is disabled (--mqtt=off)")
	}
	c.mu.RLock()
	topic, sn, account := c.reqTopic, c.sn, c.accountID
	c.mu.RUnlock()
	if sn == "" {
		return fmt.Errorf("no device_sn yet: waiting for a discovery broadcast on UDP %d", 9667)
	}
	if topic == "" {
		return fmt.Errorf("no request topic known yet")
	}
	body, err := c.envelope(cloudPayload{
		Protocol: 2, T: time.Now().UnixMilli(), DeviceSN: sn,
		AccountID: account, Data: data,
	})
	if err != nil {
		return err
	}
	log.Printf("cloud: -> %s %s", topic, compactData(data))
	return c.bus.Publish(topic, body)
}

// Beep is DP 160, a Bool. Plain JSON true, never base64: every protobuf form
// was silently dropped.
//
// It is a state, not a pulse. Confirmed on hardware 2026-07-30: the robot beeps
// until something sets it back to false, so every Beep(true) needs a Beep(false)
// to follow it. The UI gives the off switch its own button rather than a timer -
// the point of "find robot" is that it keeps going until you have found it.
func (c *Cloud) Beep(on bool) error {
	return c.SendDP(map[string]any{strconv.Itoa(dp.DPCallRobot): on})
}

// Dry fires a manual dock dry - StationRequest.manual_cmd.go_dry on DP 173.
func (c *Cloud) Dry() error {
	v, err := dp.StationAction("dry")
	if err != nil {
		return err
	}
	return c.SendDP(map[string]any{strconv.Itoa(dp.DPStation): v})
}

// Station fires any manual dock action by name.
func (c *Cloud) Station(action string) error {
	v, err := dp.StationAction(action)
	if err != nil {
		return err
	}
	return c.SendDP(map[string]any{strconv.Itoa(dp.DPStation): v})
}

// SetDryConfig writes the persistent drying mode and duration.
//
// The hour value of each duration level is firmware behaviour and is not stated
// anywhere in the protos, so this sets the level and does not claim hours. To
// map them, set a level, fire Dry, and read repair_dry_remain_seconds from
// /userdata/user_config/shared_preference.yaml - the starting value is the
// duration in seconds.
func (c *Cloud) SetDryConfig(mode, duration string) error {
	v, err := dp.DryConfig(mode, duration)
	if err != nil {
		return err
	}
	return c.SendDP(map[string]any{strconv.Itoa(dp.DPStation): v})
}

// PollStatus is the app's clock poll: DP 153 carrying a bare epoch-microsecond
// timestamp in field 1, sent every 60 s (eufy-cloud-protocol.md:245).
//
// It is here because a real client does it, not because it solves the battery
// problem - it does not. DP 163 is published only when the percentage changes,
// so no amount of polling produces one from a robot sitting at a steady charge.
// The battery comes from the chassis bus instead, see local.go.
func (c *Cloud) PollStatus() error {
	micros := uint64(time.Now().UnixMicro())
	return c.SendDP(map[string]any{
		strconv.Itoa(dp.DPWorkStatus): dp.DPValue(pb.FieldVarint(1, micros)),
	})
}

// Poll runs PollStatus on an interval until stop closes. Errors are dropped
// rather than logged: the usual one is "no client connected", once per tick,
// and the bus already reports that state.
func (c *Cloud) Poll(every time.Duration, stop <-chan struct{}) {
	t := time.NewTicker(every)
	defer t.Stop()
	for {
		select {
		case <-stop:
			return
		case <-t.C:
			_ = c.PollStatus()
		}
	}
}

// NextCmdSeq hands out the next ModeCtrlRequest sequence. Shared with the
// on-device path so the two never reuse a number.
func (c *Cloud) NextCmdSeq() uint64 { return c.cmdSeq.Add(1) }

// ModeCtrl sends a ModeCtrlRequest as DP 152 - the same message client.go
// sends over 9668, through the channel that actually works.
//
// Confirmed on hardware 2026-07-30: gohome over 9668 is received
// (`SSL RECV: 080801120408061001` in cloud_cli's log) and then silently
// dropped, no dispatch, no response, robot stationary. The identical method
// as DP 152 runs the full chain - HandleRaw -> SetModeCtrl -> ModeCtrlResponse
// result 0 - and the robot drove to its dock.
func (c *Cloud) ModeCtrl(method uint64) error {
	return c.ModeCtrlParam(method, c.NextCmdSeq(), nil)
}

// ModeCtrlParam carries the oneof Param in field 4.
func (c *Cloud) ModeCtrlParam(method, seq uint64, param []byte) error {
	body := append(pb.FieldVarint(1, method), pb.FieldVarint(2, seq)...)
	if len(param) > 0 {
		body = append(body, pb.FieldBytes(dp.ParamField(method), param)...)
	}
	return c.SendDP(map[string]any{strconv.Itoa(dp.DPModeCtrl): dp.DPValue(body)})
}

// SetVolume is DP 161, a Value 0-100. Included because the OTA announcement
// left this unit at 100 and turning it down should not need the app.
func (c *Cloud) SetVolume(level int) error {
	if level < 0 || level > 100 {
		return fmt.Errorf("volume %d out of range 0-100", level)
	}
	return c.SendDP(map[string]any{strconv.Itoa(dp.DPVolume): level})
}

// --- inbound

// Handle is the Bus message handler.
func (c *Cloud) Handle(topic string, payload []byte) {
	var msg struct {
		Head    cloudHead       `json:"head"`
		Payload json.RawMessage `json:"payload"`
	}
	if err := json.Unmarshal(payload, &msg); err != nil {
		return // not one of ours; the broker carries other traffic too
	}
	if msg.Head.ClientID == cloudClientID {
		return // our own publish, echoed back
	}

	p, err := decodePayload(msg.Payload)
	if err != nil {
		return
	}

	c.learnTopic(topic)
	c.mu.Lock()
	c.lastSeen = time.Now()
	if p.AccountID != "" {
		c.accountID = p.AccountID
	}
	if p.DeviceSN != "" {
		c.sn = p.DeviceSN
	}
	c.mu.Unlock()

	switch p.Protocol {
	case 1:
		c.applyReport(p.Data)
	case 7:
		c.answerRequest(p)
	case 10:
		log.Printf("cloud: OTA progress %s", compactData(p.Data))
	case 4, 5:
		// online/offline notices
	default:
		log.Printf("cloud: protocol %d %s", p.Protocol, compactData(p.Data))
	}
}

// decodePayload accepts both directions' encodings: the robot nests the payload
// object, the cloud and the app send it as a string.
func decodePayload(raw json.RawMessage) (cloudPayload, error) {
	var p cloudPayload
	if len(raw) == 0 {
		return p, fmt.Errorf("empty payload")
	}
	if raw[0] == '"' {
		var s string
		if err := json.Unmarshal(raw, &s); err != nil {
			return p, err
		}
		return p, json.Unmarshal([]byte(s), &p)
	}
	return p, json.Unmarshal(raw, &p)
}

// learnTopic takes the reply topic from live traffic rather than assuming it.
// After a factory reset the robot loses the app-name segment and publishes on
// cmd//T2351/<sn>/res, so a hard-coded eufy_home would talk into the void.
func (c *Cloud) learnTopic(topic string) {
	if !strings.HasSuffix(topic, "/res") {
		return
	}
	req := strings.TrimSuffix(topic, "/res") + "/req"
	c.mu.Lock()
	changed := c.resTopic != topic
	c.resTopic, c.reqTopic = topic, req
	first := !c.announced[topic]
	c.announced[topic] = true
	sn, app := c.sn, c.appDefault
	c.mu.Unlock()

	if changed {
		log.Printf("cloud: robot publishes on %s, replying on %s", topic, req)
	}
	if first && c.bus != nil {
		// Protocol 4, the device-online notice. Sent on the
		// topic the device published on, which is what the capture showed.
		if body, err := c.envelope(cloudPayload{
			Protocol: 4, T: time.Now().UnixMilli(), DeviceSN: sn,
			Data: map[string]any{"app_id": app},
		}); err == nil {
			_ = c.bus.Publish(topic, body)
		}
	}
}

func (c *Cloud) applyReport(data map[string]any) {
	c.mu.Lock()
	defer c.mu.Unlock()
	for k, v := range data {
		id, err := strconv.Atoi(k)
		if err != nil {
			continue // protocol 4/5/7/8 put req_type and app_id here, not DPs
		}
		c.dps[id] = dp.DecodeDP(id, v)
		switch id {
		case dp.DPBatLevel:
			if n, ok := v.(float64); ok {
				c.battery = int(n)
			}
		case dp.DPWorkStatus:
			// only when this report carried a state - they are partial
			if state, ok := dp.WorkStateOf(v); ok {
				c.workState = state
			}
		}
	}
}

// answerRequest handles protocol 7. cloud_time is the only request seen, and
// answering it is not optional: the robot waits on it during boot.
func (c *Cloud) answerRequest(p cloudPayload) {
	req, _ := p.Data["req_type"].(string)
	if req != "cloud_time" {
		log.Printf("cloud: unhandled protocol 7 request %q", req)
		return
	}
	c.mu.RLock()
	topic, sn := c.reqTopic, c.sn
	c.mu.RUnlock()
	ms := time.Now().UnixMilli()
	body, err := c.envelope(cloudPayload{
		Protocol: 8, T: ms, DeviceSN: sn, AccountID: p.AccountID,
		Data: map[string]any{"req_type": "cloud_time", "res_type": "Int", "res_value": ms},
	})
	if err != nil {
		return
	}
	if c.bus == nil {
		return
	}
	if err := c.bus.Publish(topic, body); err != nil {
		c.mu.Lock()
		c.lastErr = err.Error()
		c.mu.Unlock()
		return
	}
	log.Printf("cloud: cloud_time answered, res_value=%d", ms)
}

func compactData(data map[string]any) string {
	b, err := json.Marshal(data)
	if err != nil {
		return "?"
	}
	if len(b) > 200 {
		return string(b[:200]) + "..."
	}
	return string(b)
}
