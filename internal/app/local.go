package app

// On-device control and telemetry, bypassing the cloud protocol entirely.
//
// The robot runs two IPC layers. `akostopic` reads the akos topic bus, which
// carries sensor and state telemetry - 537 published topics, against the 31
// data points the MQTT channel exposes. `audio_cli` and friends reach the other
// layer, eros::mds, which is where commands actually land: cloud_cli and
// device_svc are both mds participants and neither is an akos node, which is
// why publishing to akos command topics does nothing at all.
//
// Both are reached by shelling out. Speaking mds natively would mean
// reimplementing a shared-memory transport and its protobuf channel registry
// for no gain: the vendor's own CLIs are on the device, they are the reference
// implementation, and a subprocess is a rounding error next to a robot.
//
// Everything here only works when x10-ctl runs ON the robot. The bus is shared
// memory. Off-device this whole file is inert and the MQTT path carries on.

import (
	"bufio"
	"context"
	"encoding/hex"
	"encoding/json"
	"fmt"
	"github.com/dhumpf/x10-ctl/internal/dp"
	"github.com/dhumpf/x10-ctl/internal/pb"
	"log"
	"os"
	"os/exec"
	"strconv"
	"strings"
	"sync"
	"syscall"
	"time"
)

const (
	appProfile   = "/etc/profile.d/eufy_app.sh"
	akosTopic    = "/app/bin/akostopic"
	audioCLI     = "/app/bin/audio_cli"
	mdsPub       = "/app/bin/mds_pub"
	batteryTopic = "/topic/chassis/batteryinfo"

	// The lidar's own channels. /topic/chassis/laser_scan, which an earlier
	// version polled, is only a *name* defined in global_topics.yaml - no node
	// ever advertises it, so it read empty forever. The scans are on mds.
	mdsBatteryInfo  = "eros.proto.msg.chassis.BatteryInfo"
	mdsLaserScan    = "eros.proto.msg.sensor.LaserScan"
	mdsLaserControl = "eros.proto.msg.sensor.LaserControl"
	mdsCameraCtrl   = "eros.proto.msg.sensor.camera.Control"

	// The internal-bus channel that both existing transports converge on:
	// cloud_cli owns the 9668 listener AND the MQTT client, and forwards mode
	// control here. robotapp subscribes via ReaderClosure<...ModeCtrlRequest>.
	// Writing it directly bypasses both.
	mdsModeCtrlChannel = "eros.proto.msg.app.ModeCtrlRequest"

	// What the robot is doing, published by node "cloud" on every change.
	// Structurally identical to DP 153's body, so workStateOf decodes both.
	mdsStatusNotify = "eros.proto.msg.app.StatusUpdateNotify"

	// The robot's own cloud connection, published by cloud_robotdev. Nothing to
	// do with x10-ctl's broker client - this is whether cloud_cli is talking to
	// whatever cloud the robot is configured to reach.
	mdsCloudNetwork = "eros.proto.msg.app.CloudNetwork"

	// The beep. dev.audio replays this file until something stops it, which is
	// why "find robot" needs an explicit off and not a timer.
	beepSound = "/oem/lang/en_us_male/main/0001.mp3"
)

// Local is the on-device surface. Zero value is "not on the robot".
type Local struct {
	available bool

	mu        sync.RWMutex
	battery   Battery
	seen      time.Time
	lastErr   string
	workState string
	pose      *RobotPose
	poseSeen  time.Time
	poseWant  time.Time

	// Presence sensors, remembered: SensorData is partial and a record that
	// omits them is not a record saying they are gone.
	dustBox, waterBox, mopLeft, mopRight *bool

	// Last time the charge state was non-zero, which is the last time the robot
	// was demonstrably on its dock. Only meaningful once a battery report has
	// arrived; zero means "never seen charging", not "never docked".
	lastOnDock time.Time

	// Odom continuity tracking; see poseJumped in pose.go.
	lastX, lastY float64
	haveLast     bool
	odomDist     float64
	odomJumpAt   time.Time
}

// Battery is node_sensor_msgs/BatteryInfo, the fields worth showing. The MQTT
// data point carries only Level; the rest exists solely on the bus.
type Battery struct {
	Level        int  `json:"level"`
	Millivolts   int  `json:"millivolts"`
	ChargeState  int  `json:"charge_state"`
	BatteryState int  `json:"battery_state"`
	ChargeError  bool `json:"charge_error"`
	DischargeErr bool `json:"discharge_error"`
	ChipError    bool `json:"charging_chip_error"`
	Valid        bool `json:"valid"`
}

// BatteryInfo field numbers. UNVERIFIED: counted out of the message definition
// in akos-bus.md assuming an unpopulated header is not emitted, so charge_state
// lands at 1. That is exactly the guess LaserPowerProxy caught this project
// making before - it put the payload at 1 and the wire said 2. Confirm against
// a live record with cmd/mdsdump, cross-checked against what akostopic prints
// for the same battery, before trusting a reading from this.
const (
	batteryFieldChargeState    = 1
	batteryFieldBatteryState   = 2
	batteryFieldLevel          = 4
	batteryFieldMillivolts     = 5
	batteryFieldChipError      = 6
	batteryFieldChargeError    = 8
	batteryFieldDischargeError = 9
)

// batteryInfo mirrors the JSON akostopic prints. Separate from Battery so the
// wire shape and the shape we serve can drift independently.
//
// The proto declares charging_chip_error and friends as bool, but akostopic
// renders them as 0 and 1, not true and false - the same integer-bool
// convention its `pub` side insists on. Decoding them as Go bool fails the
// whole message, and since a failed decode is skipped, the effect is a battery
// that silently never updates.
type batteryInfo struct {
	ChargeState       int `json:"charge_state"`
	BatteryState      int `json:"battery_state"`
	BatteryLevel      int `json:"battery_level"`
	BatteryVol        int `json:"battery_vol"`
	ChargingChipError int `json:"charging_chip_error"`
	ChargeError       int `json:"charge_error"`
	DischargeError    int `json:"discharge_error"`
}

// NewLocal reports whether the on-device tools are present. It does not run
// anything: detection is a stat, so an off-device run costs nothing.
func NewLocal() *Local {
	l := &Local{}
	for _, p := range []string{appProfile, akosTopic} {
		if _, err := os.Stat(p); err != nil {
			return l
		}
	}
	l.available = true
	return l
}

func (l *Local) Available() bool { return l != nil && l.available }

// run executes an /app/bin tool with the robot's own environment.
//
// Sourcing the profile is not optional and not cosmetic. With
// MDS_CHANNEL_FILEPATH unset every mds tool still returns 0 and still logs
// "CreateMessage success", then silently drops the message because no channel
// config was loaded. Setting only LD_LIBRARY_PATH looks like it works and
// does nothing.
// Cancelling has to kill the process group, not the direct child. A read that
// wrapped itself in `timeout 8` left an `akostopic echo` running for 19 hours
// on 2026-08-06: busybox timeout sends SIGTERM, akostopic does not take the
// hint, timeout exits anyway and the subscriber is reparented to init, still
// holding its 36 MB. Killing the group with SIGKILL and letting WaitDelay
// backstop a child that holds the pipe open leaves nothing behind.
func (l *Local) run(ctx context.Context, argv string) ([]byte, error) {
	cmd := exec.CommandContext(ctx, "/bin/sh", "-c",
		". "+appProfile+" >/dev/null 2>&1; export LD_LIBRARY_PATH=/app/lib:$LD_LIBRARY_PATH; exec "+argv)
	cmd.SysProcAttr = &syscall.SysProcAttr{Setpgid: true}
	cmd.Cancel = func() error {
		if cmd.Process == nil {
			return nil
		}
		return syscall.Kill(-cmd.Process.Pid, syscall.SIGKILL)
	}
	cmd.WaitDelay = 2 * time.Second
	return cmd.CombinedOutput()
}

// --- control

// Beep drives DPID_CALLING_ROBOT's real destination directly:
// eros.proto.msg.audio.Control, method 1 to play and 2 to stop, handled by
// dev.audio inside device_svc. No broker, no cloud, no data point.
//
// A bare numeric id is not accepted by audio_cli; the argument has to be an
// absolute path.
func (l *Local) Beep(ctx context.Context, on bool) error {
	if !l.Available() {
		return fmt.Errorf("not running on the robot")
	}
	argv := audioCLI + " -s"
	if on {
		argv = audioCLI + " -o " + beepSound
	}
	out, err := l.run(ctx, argv)
	if err != nil {
		return fmt.Errorf("audio_cli: %w: %s", err, strings.TrimSpace(string(out)))
	}
	return nil
}

// ModeCtrl publishes a ModeCtrlRequest onto the internal bus, which is the
// whole point of this file: no broker, no cloud, no data point.
//
//	eros.proto.msg.app.ModeCtrlRequest {
//	  eros.proto.common.Header    header = 1;   // omitted, see below
//	  proto.cloud.ModeCtrlRequest req    = 2;   // { method = 1, seq = 2 }
//	}
//
// The channel name goes to mds_pub verbatim; it hashes the name with
// _Hash_bytes(name, 0xc70f6907) to get the channel id, whose low 32 bits are
// the SysV key. Payload is the serialized message, hex encoded.
//
// The header is left out and that turns out to be fine - robotapp accepts the
// message with field 1 absent, so chassis_pub's SetMessageHeader is a
// convention rather than a requirement.
//
// Confirmed on hardware 2026-07-30: stop and gohome both produced
// `ModeCtrlResponse method N result 0` with no HandleCmdMqttMessage and no
// SSL RECV anywhere in the log - neither MQTT nor 9668 was involved.
func (l *Local) ModeCtrl(ctx context.Context, method, seq uint64) error {
	return l.ModeCtrlParam(ctx, method, seq, nil)
}

// ModeCtrlParam is ModeCtrl with the oneof Param - room selections, zones, a
// target point. Seven methods require one and mean nothing without it.
//
// The field number is the method's own, from paramField: rooms 4, zones 5,
// spot 6. Hardcoding 4 here sent zones as f4{f5{...}} and spot as f4{f6{...}},
// which the robot dropped in silence on both channels.
func (l *Local) ModeCtrlParam(ctx context.Context, method, seq uint64, param []byte) error {
	if !l.Available() {
		return fmt.Errorf("not running on the robot")
	}
	req := append(pb.FieldVarint(1, method), pb.FieldVarint(2, seq)...)
	if len(param) > 0 {
		req = append(req, pb.FieldBytes(dp.ParamField(method), param)...)
	}
	return l.mdsPublish(ctx, mdsModeCtrlChannel, pb.FieldBytes(2, req))
}

// LaserScan is one lidar sweep, decoded off eros.proto.msg.sensor.LaserScan.
//
// Field numbers were read from live records rather than from a schema: the mds
// protos are compiled LITE_RUNTIME, so unlike the cloud protos they leave no
// FileDescriptor in the binary to recover.
//
//	f2  float  angle_min        -pi
//	f3  float  angle_max        +pi
//	f4  float  angle_increment   0.011 rad
//	f7  float  range_min         0.08 m
//	f8  float  range_max         8.0 m
//	f9  uint64 stamp             CLOCK_MONOTONIC nanoseconds
//	f10 packed float ranges      571 points, 0 meaning no return
//	f11 packed float intensities
//	f13 packed float angles      per-point, so f4 need not be trusted
type LaserScan struct {
	AngleMin  float64   `json:"angle_min"`
	AngleMax  float64   `json:"angle_max"`
	AngleIncr float64   `json:"angle_increment"`
	RangeMin  float64   `json:"range_min"`
	RangeMax  float64   `json:"range_max"`
	Ranges    []float64 `json:"ranges"`
	Angles    []float64 `json:"angles"`
	Intensity []float64 `json:"intensities"`
	Stamp     float64   `json:"stamp"`
	Age       float64   `json:"age"`
}

// Scan reads the newest lidar sweep straight out of shared memory.
//
// Sampled on demand rather than streamed. The lidar only spins while it has
// power, and power is off whenever the robot is docked and idle - see
// LaserPower - so a permanent subscriber would mostly idle empty.
func (l *Local) Scan(ctx context.Context) (*LaserScan, error) {
	if !l.Available() {
		return nil, fmt.Errorf("not running on the robot")
	}
	rec, err := mdsNewest(mdsLaserScan)
	if err != nil {
		return nil, err
	}
	sc := &LaserScan{}
	for _, f := range pb.ParseFields(rec) {
		switch f.Num {
		case 2:
			sc.AngleMin = pb.Float32(f)
		case 3:
			sc.AngleMax = pb.Float32(f)
		case 4:
			sc.AngleIncr = pb.Float32(f)
		case 7:
			sc.RangeMin = pb.Float32(f)
		case 8:
			sc.RangeMax = pb.Float32(f)
		case 9:
			sc.Stamp = float64(f.Val) / 1e9
		case 10:
			sc.Ranges = pb.PackedFloat32(f.Data)
		case 11:
			sc.Intensity = pb.PackedFloat32(f.Data)
		case 13:
			sc.Angles = pb.PackedFloat32(f.Data)
		}
	}
	if len(sc.Ranges) == 0 {
		return nil, fmt.Errorf("scan carried no ranges - is the laser powered?")
	}
	if age, ok := frameAge(sc.Stamp); ok {
		sc.Age = age
	}
	return sc, nil
}

// LaserPower switches the lidar on or off without starting a cleaning task.
//
// The obvious way to get the lidar spinning is to start a mode that needs it,
// but that also runs the brushes and drops the mops, which is no good for a
// robot parked on a wooden floor. This is the switch the stack uses internally:
//
//	robotapp --LaserControl--> lidar_service --LaserPowerProxy--> control
//
// Publishing LaserControl is what robotapp itself does (its records carry
// node "app-cmd", frame_id "base_link", name "lidarctrl"), so every guard in
// lidar_service and the chassis stays in front of us. Publishing the
// PowerProxy instead would skip that layer and drive the hardware directly.
//
//	LaserControl { Header header = 1; Power power = 2 }
//	Power        { uint32 on = 1 }
//
// Confirmed on hardware 2026-07-30: `12020801` produced "lidar power request:
// 1", "laser power status update from 0 to 1" and a 6.9 Hz spin, on the dock,
// with no cleaning mode and no mop.
func (l *Local) LaserPower(ctx context.Context, on bool) error {
	return l.mdsPublish(ctx, mdsLaserControl, pb.FieldBytes(2, onOff(on)))
}

// CameraStream holds the RGB stream open under our own owner id.
//
// This is why the camera looked stale: camera_svc only runs the sensor while
// somebody has asked it to. robotapp asks while the app's live view is open and
// releases afterwards, leaving the last frame sitting in shared memory for
// however long - four minutes, in the case that prompted this.
//
//	camera.Control { Header header = 1; Stream stream = 2; Light light = 3 }
//	Stream         { uint32 on = 1; string id = 2 }
//
// The id is an owner tag, not a device selector, so using our own leaves
// robotapp's request untouched either way. Confirmed on hardware: camera_svc
// logged `msg->stream().id()=x10ctl, sw=1`, brought up the MIPI CSI link and
// started writing at cycle 0.0333 - 30 fps.
func (l *Local) CameraStream(ctx context.Context, on bool) error {
	body := onOff(on)
	body = append(body, pb.FieldBytes(2, []byte(cameraOwner))...)
	return l.mdsPublish(ctx, mdsCameraCtrl, pb.FieldBytes(2, body))
}

// CameraLight sets the camera's supplement LED.
//
//	camera.Control { Header header = 1; Stream stream = 2; Light light = 3 }
//	Light          { uint32 level = 1 }
//
// level is nominally a percentage: camera_gc2053 logs `CtrlLight level=N,
// enable=E` and derives enable from level being non-zero. Values 1, 2, 3, 5,
// 100 and 0 are all accepted.
//
// The LED does not actually dim, though - checked by eye, every non-zero level
// looks the same. So the level is real in the protocol and not in the hardware,
// and callers should treat this as a switch.
func (l *Local) CameraLight(ctx context.Context, level int) error {
	if level < 0 {
		level = 0
	}
	if level > 100 {
		level = 100
	}
	var body []byte
	if level > 0 {
		body = pb.FieldVarint(1, uint64(level))
	}
	return l.mdsPublish(ctx, mdsCameraCtrl, pb.FieldBytes(3, body))
}

// onOff encodes a one-field submessage body. proto3 omits a zero varint, and
// the robot's own records do exactly that: power-off is an empty submessage.
func onOff(on bool) []byte {
	if !on {
		return nil
	}
	return pb.FieldVarint(1, 1)
}

const cameraOwner = "x10ctl"

// mdsPublish sends one message on a channel via the vendor's own publisher.
//
// Reads go straight to shared memory (see mds.go), but writes do not: mds_pub
// fills in the Header, and a publisher has to be registered with the channel
// for subscribers to be woken. Shelling out is a rounding error next to a robot.
func (l *Local) mdsPublish(ctx context.Context, channel string, payload []byte) error {
	if !l.Available() {
		return fmt.Errorf("not running on the robot")
	}
	arg := channel + "-" + hex.EncodeToString(payload)
	out, err := l.run(ctx, mdsPub+" '"+arg+"'")
	if err != nil {
		return fmt.Errorf("mds_pub %s: %w: %s", channel, err, strings.TrimSpace(string(out)))
	}
	if s := string(out); strings.Contains(s, "failed") || strings.Contains(s, "ERROR") {
		return fmt.Errorf("mds_pub %s did not transmit: %s", channel, strings.TrimSpace(s))
	}
	return nil
}

// WorkStatus reports what the robot is doing, as last seen on the bus.
//
// Cached rather than read on demand, and the reason is the shape of the data
// rather than cost. Reports are partial: an off-dock one carries fields 14 and
// 15 with no state at all, so the newest record routinely says nothing about
// what the robot is doing. Walking back through the ring for the last record
// that did is unreliable too - the ring wraps, and after a wrap there is no way
// to find record boundaries from a cold start.
//
// So the state is accumulated by WatchStatus and remembered, which is also what
// the DP path had to do for exactly the same reason.
//
// Remembering is also how it goes wrong. A state outlives the thing it
// describes: the robot finishes, docks, and the docking report carries no state
// field, so the cache still says "cleaning" with the robot sat on its dock at
// 100%. That is what reconcileStatus is for.
func (l *Local) WorkStatus() (string, bool) {
	st, _, ok := l.WorkStatusDetail()
	return st, ok
}

// WorkStatusDetail is WorkStatus plus whether the answer came from the charge
// state rather than from a report. Worth surfacing: a derived answer is a
// statement about where the robot is, not about what it is doing.
func (l *Local) WorkStatusDetail() (state string, derived bool, ok bool) {
	if !l.Available() {
		return "", false, false
	}
	l.mu.RLock()
	defer l.mu.RUnlock()
	st, offFor := l.workState, time.Duration(0)
	if !l.lastOnDock.IsZero() {
		offFor = time.Since(l.lastOnDock)
	}
	fixed, derived := reconcileStatus(st, l.battery, offFor)
	return fixed, derived, fixed != ""
}

// ReportedWorkStatus is the raw cached state, before any reconciliation. Shown
// beside a derived answer so the disagreement is visible rather than hidden.
func (l *Local) ReportedWorkStatus() (string, bool) {
	l.mu.RLock()
	defer l.mu.RUnlock()
	return l.workState, l.workState != ""
}

// dockGrace is how long the charge state has to stay at zero before a cached
// "docked" is treated as out of date.
//
// One reading is not enough. Charging is not continuous once the pack is full -
// the charge state observed on the dock at 100% is 2, not 1 - so a controller
// that drops to 0 for a cycle between top-ups would otherwise look exactly like
// a robot that had driven away. Leaving the dock takes far longer than this, so
// nothing real is delayed by waiting.
const dockGrace = 30 * time.Second

// reconcileStatus settles the cached state against the charge state.
//
// charge_state is the one signal here that is published unconditionally, every
// 2 s, whether or not anything changed. Values seen on this robot: 0 off the
// dock (recorded at 97%), 2 on the dock with the battery full. Charging cannot
// happen anywhere but the dock, so non-zero means docked, and that is the whole
// basis for what follows.
//
// It deliberately does not do more than that. The charge state says nothing
// about what the robot is doing once it is off the dock, so an off-dock state
// is passed through untouched, and an on-dock state that is not "cleaning" is
// left alone too - washing and drying both happen on the dock and report
// states this code has never seen a name for. The only claim being corrected is
// the one that cannot be true: cleaning, on the dock.
func reconcileStatus(state string, b Battery, offDockFor time.Duration) (string, bool) {
	if !b.Valid {
		return state, false
	}
	if b.ChargeState > 0 {
		if state == "" || state == "cleaning" {
			return "docked", true
		}
		return state, false
	}
	if state == "docked" && offDockFor > dockGrace {
		// Off the dock, and the last report we have says otherwise. What it is
		// doing is genuinely unknown; saying so beats repeating a stale answer.
		return "off_dock", true
	}
	return state, false
}

// WatchStatus follows app.StatusUpdateNotify until stop closes.
//
// This is what replaced polling DP 153. The data point is reported on change
// and only to whoever is attached at the time, so a client that connects late
// sees nothing; the bus channel holds the last report regardless of who was
// listening. Its field 2 is the same message the data point wraps.
func (l *Local) WatchStatus(stop <-chan struct{}) {
	if !l.Available() {
		return
	}
	tick := time.NewTicker(2 * time.Second)
	defer tick.Stop()
	var lastSeq uint64
	var seenSeq bool
	for {
		if rec, err := mdsNewest(mdsStatusNotify); err == nil {
			seq, hasSeq := recordSeq(rec)
			fresh := !hasSeq || !seenSeq || seq != lastSeq
			if hasSeq {
				lastSeq, seenSeq = seq, true
			}
			if st, ok := statusOf(rec); ok && fresh {
				l.mu.Lock()
				l.workState = st
				l.mu.Unlock()
			}
		}
		select {
		case <-stop:
			return
		case <-tick.C:
		}
	}
}

// recordSeq is the publisher's own counter, header field 1.3. The channel stops
// publishing when a run ends and mdsNewest then returns that last record for
// ever, so an unchanged seq means history rather than state.
func recordSeq(rec []byte) (uint64, bool) {
	for _, f := range pb.ParseFields(rec) {
		if f.Num != 1 || f.Wire != 2 {
			continue
		}
		for _, g := range pb.ParseFields(f.Data) {
			if g.Num == 3 && g.Wire == 0 {
				return g.Val, true
			}
		}
	}
	return 0, false
}

// statusOf pulls the work state out of one StatusUpdateNotify record, if that
// record carried one.
func statusOf(rec []byte) (string, bool) {
	for _, f := range pb.ParseFields(rec) {
		if f.Num != 2 || f.Wire != 2 {
			continue
		}
		// Field 6 is present while a clean is paused. Only trust it on a record
		// the publisher has actually just written - see recordSeq.
		paused := false
		for _, g := range pb.ParseFields(f.Data) {
			if g.Num == 6 && g.Wire == 2 {
				paused = true
			}
		}
		for _, g := range pb.ParseFields(f.Data) {
			if g.Num == 2 && g.Wire == 0 {
				if paused && g.Val == 5 {
					return "paused", true
				}
				if name, ok := dp.WorkStates[g.Val]; ok {
					return name, true
				}
				return "state " + strconv.FormatUint(g.Val, 10), true
			}
		}
	}
	return "", false
}

// CloudLink reports whether the robot itself is talking to its cloud.
//
// Worth showing in place of x10-ctl's own broker state, which since the broker
// was dropped only ever said "off" and told nobody anything. This is the robot's
// link, and it stays up whether or not we use one.
func (l *Local) CloudLink() (int, bool) {
	if !l.Available() {
		return 0, false
	}
	rec, err := mdsNewest(mdsCloudNetwork)
	if err != nil {
		return 0, false
	}
	for _, f := range pb.ParseFields(rec) {
		if f.Num == 2 && f.Wire == 0 {
			return int(f.Val), true
		}
	}
	return 0, false
}

// Languages lists the installed voice packs. Read-only, and the one audio_cli
// call that needs no channel config.
func (l *Local) Languages(ctx context.Context) (string, error) {
	if !l.Available() {
		return "", fmt.Errorf("not running on the robot")
	}
	out, err := l.run(ctx, audioCLI+" -p")
	return strings.TrimSpace(string(out)), err
}

// --- telemetry

func (l *Local) Battery() Battery {
	l.mu.RLock()
	defer l.mu.RUnlock()
	return l.battery
}

func (l *Local) Status() map[string]any {
	l.mu.RLock()
	defer l.mu.RUnlock()
	st := map[string]any{"available": l.available}
	if !l.available {
		return st
	}
	st["battery"] = l.battery
	if !l.seen.IsZero() {
		st["last_seen"] = l.seen
	}
	if l.lastErr != "" {
		st["last_error"] = l.lastErr
	}
	return st
}

// WatchBattery streams BatteryInfo off the chassis bus until stop closes.
//
// This is the whole reason the bus is worth shelling out to. DP 163 is
// published only when the percentage changes and only to whoever is attached at
// the time, so a client that starts later can wait hours and see nothing. The
// bus publishes every 2 s regardless, and carries voltage and four fault flags
// the data point does not have.
//
// This was moved to a read-only mds poll on 2026-08-06 to be rid of the 36 MB
// subprocess, and moved back the same day. The mds records are partial: sampled
// for 24 s against a cleaning robot, chassis.BatteryInfo emitted the level on
// its own in one record and two flags on their own in another, and carried no
// voltage and no charge state at all. akostopic renders the whole struct every
// 2 s. Polling the newest record would have reported millivolts 0 and
// charge_state 0 - values the robot never published - so the subprocess stays
// until a merge across records is shown to reconstruct the full struct.
func (l *Local) WatchBattery(stop <-chan struct{}) {
	if !l.Available() {
		return
	}
	for {
		select {
		case <-stop:
			return
		default:
		}
		if err := l.streamBattery(stop); err != nil {
			l.mu.Lock()
			l.lastErr = err.Error()
			l.mu.Unlock()
			log.Printf("local: battery stream: %v (retrying in 10s)", err)
		}
		select {
		case <-stop:
			return
		case <-time.After(10 * time.Second):
		}
	}
}

func (l *Local) streamBattery(stop <-chan struct{}) error {
	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()
	go func() {
		<-stop
		cancel()
	}()

	cmd := exec.CommandContext(ctx, "/bin/sh", "-c",
		". "+appProfile+" >/dev/null 2>&1; export LD_LIBRARY_PATH=/app/lib:$LD_LIBRARY_PATH; exec "+
			akosTopic+" echo "+batteryTopic)
	out, err := cmd.StdoutPipe()
	if err != nil {
		return err
	}
	cmd.Stderr = nil
	if err := cmd.Start(); err != nil {
		return err
	}
	defer func() {
		_ = cmd.Process.Kill()
		_ = cmd.Wait()
	}()

	sc := bufio.NewScanner(out)
	sc.Buffer(make([]byte, 0, 64*1024), 1<<20)
	got := false
	for sc.Scan() {
		info, ok := parseAkosJSON(sc.Text())
		if !ok {
			continue
		}
		var b batteryInfo
		if json.Unmarshal(info, &b) != nil {
			continue
		}
		got = true
		l.mu.Lock()
		l.battery = Battery{
			Level: b.BatteryLevel, Millivolts: b.BatteryVol,
			ChargeState: b.ChargeState, BatteryState: b.BatteryState,
			ChargeError: b.ChargeError != 0, DischargeErr: b.DischargeError != 0,
			ChipError: b.ChargingChipError != 0, Valid: true,
		}
		l.seen, l.lastErr = time.Now(), ""
		if b.ChargeState > 0 {
			l.lastOnDock = l.seen
		}
		l.mu.Unlock()
	}
	if !got {
		// akostopic prints its shared-memory attach banner and then, if the
		// domain is wrong, nothing at all. Silence is the symptom.
		return fmt.Errorf("no messages on %s - is AKOS_DOMAIN_ID right?", batteryTopic)
	}
	return ctx.Err()
}

// parseAkosJSON pulls the payload out of one akostopic echo line:
//
//	[2026-07-30 14:21:19.536] /topic/chassis/batteryinfo[node_sensor_msgs/BatteryInfo]->>[31]{"header":...}
//
// The banner lines carry no "->>" and are skipped.
func parseAkosJSON(line string) (json.RawMessage, bool) {
	i := strings.Index(line, "->>")
	if i < 0 {
		return nil, false
	}
	j := strings.Index(line[i:], "{")
	if j < 0 {
		return nil, false
	}
	return json.RawMessage(line[i+j:]), true
}
