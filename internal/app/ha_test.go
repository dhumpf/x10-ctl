package app

import (
	"encoding/base64"
	"encoding/hex"
	"encoding/json"
	"github.com/dhumpf/x10-ctl/internal/client"
	"github.com/dhumpf/x10-ctl/internal/cloud"
	"github.com/dhumpf/x10-ctl/internal/dp"
	"github.com/dhumpf/x10-ctl/internal/mqtt"
	"github.com/dhumpf/x10-ctl/internal/pb"
	"os"
	"strings"
	"testing"
)

func testBridge() *haBridge {
	return newHABridge(&server{}, "AMP00X0E00000000")
}

// HA rejects a config carrying a feature string outside this set, and rejects
// the whole device payload with it.
func TestSupportedFeaturesAreLegal(t *testing.T) {
	legal := map[string]bool{
		"start": true, "stop": true, "pause": true, "return_home": true,
		"status": true, "locate": true, "clean_spot": true,
		"fan_speed": true, "send_command": true,
	}
	for _, f := range haFeatures {
		if !legal[f] {
			t.Errorf("%q is not a supported_features string HA accepts", f)
		}
	}
}

// Every declared feature needs its topic, or the button appears and does
// nothing. fan_speed is the one that carries a separate topic key.
func TestDeclaredFeaturesHaveTopics(t *testing.T) {
	v := testBridge().components()["vacuum"].(map[string]any)
	for _, f := range haFeatures {
		switch f {
		case "fan_speed":
			if _, ok := v["set_fan_speed_topic"]; !ok {
				t.Error("fan_speed declared with no set_fan_speed_topic")
			}
			if _, ok := v["fan_speed_list"]; !ok {
				t.Error("fan_speed declared with no fan_speed_list")
			}
		case "send_command":
			if _, ok := v["send_command_topic"]; !ok {
				t.Error("send_command declared with no send_command_topic")
			}
		}
	}
	if _, ok := v["state_topic"]; !ok {
		t.Error("vacuum has no state_topic")
	}
	if _, ok := v["command_topic"]; !ok {
		t.Error("vacuum has no command_topic")
	}
}

// The legacy schema key was removed from core in 2024.8. Emitting it now is an
// unknown key on the config.
func TestNoLegacySchemaKey(t *testing.T) {
	for name, c := range testBridge().components() {
		if _, ok := c.(map[string]any)["schema"]; ok {
			t.Errorf("%s carries a schema key; it was removed from core in 2024.8", name)
		}
	}
}

// Battery stopped being a vacuum property in HA 2026.8. It has to be its own
// sensor or it silently stops rendering.
func TestBatteryIsItsOwnSensor(t *testing.T) {
	cmps := testBridge().components()
	v := cmps["vacuum"].(map[string]any)
	for _, k := range []string{"battery_level", "battery_icon", "battery_level_topic"} {
		if _, ok := v[k]; ok {
			t.Errorf("vacuum carries %s; removed from core in 2026.8", k)
		}
	}
	b, ok := cmps["battery"].(map[string]any)
	if !ok {
		t.Fatal("no separate battery sensor")
	}
	if b["dev_cla"] != "battery" || b["unit_of_meas"] != "%" {
		t.Errorf("battery sensor is not a battery device class: %v", b)
	}
}

// Duplicate unique_ids make HA drop every entity after the first.
func TestUniqueIDsAreUnique(t *testing.T) {
	seen := map[string]string{}
	for name, c := range testBridge().components() {
		id, _ := c.(map[string]any)["unique_id"].(string)
		if id == "" {
			t.Errorf("%s has no unique_id", name)
			continue
		}
		if prev, dup := seen[id]; dup {
			t.Errorf("unique_id %q used by both %s and %s", id, prev, name)
		}
		seen[id] = name
	}
}

// Every state_topic and command_topic has to sit under our own root, or the
// bridge subscribes to one thing and HA publishes to another.
func TestTopicsStayUnderTheBase(t *testing.T) {
	h := testBridge()
	for name, c := range h.components() {
		m := c.(map[string]any)
		for _, k := range []string{"state_topic", "command_topic", "set_fan_speed_topic", "send_command_topic", "avty_t"} {
			v, ok := m[k].(string)
			if !ok {
				continue
			}
			if !strings.HasPrefix(v, h.base+"/") {
				t.Errorf("%s.%s = %q escapes base %q", name, k, v, h.base)
			}
		}
	}
}

// The bridge subscribes to base/cmd/# only, so a command topic outside that
// prefix is never delivered.
func TestCommandTopicsAreSubscribed(t *testing.T) {
	h := testBridge()
	sub := h.base + "/cmd/"
	for name, c := range h.components() {
		m := c.(map[string]any)
		for _, k := range []string{"command_topic", "set_fan_speed_topic", "send_command_topic"} {
			v, ok := m[k].(string)
			if !ok {
				continue
			}
			if !strings.HasPrefix(v, sub) {
				t.Errorf("%s.%s = %q is not under %q, so it is never received", name, k, v, sub)
			}
		}
	}
}

func TestDiscoveryPayloadShape(t *testing.T) {
	h := testBridge()
	body, err := json.Marshal(map[string]any{
		"dev": h.device(), "o": map[string]any{"name": "x10-ctl"}, "cmps": h.components(),
	})
	if err != nil {
		t.Fatal(err)
	}
	var back map[string]any
	if err := json.Unmarshal(body, &back); err != nil {
		t.Fatal(err)
	}
	dev, ok := back["dev"].(map[string]any)
	if !ok {
		t.Fatal("no dev block")
	}
	if _, ok := dev["ids"]; !ok {
		t.Error("device block has no identifiers; entities will not group")
	}
	// `ori` is not an abbreviation core knows; the origin key is `o`.
	if _, bad := back["ori"]; bad {
		t.Error("origin emitted as ori; core only understands o")
	}
	if _, ok := back["o"]; !ok {
		t.Error("no origin block")
	}
}

// Only the six activity strings are legal. Anything else lands as unknown.
func TestActivityMappingIsLegal(t *testing.T) {
	legal := map[string]bool{
		"cleaning": true, "docked": true, "idle": true,
		"paused": true, "returning": true, "error": true,
	}
	for from, to := range haActivity {
		if !legal[to] {
			t.Errorf("work state %q maps to %q, which HA does not accept", from, to)
		}
	}
}

// Every button has to resolve to something that will run, or it appears in the
// dashboard and fails silently on press.
func TestButtonsResolve(t *testing.T) {
	for _, b := range haButtons {
		_, isDP := dpCommands[b.cmd]
		_, isMethod := client.Methods[b.cmd]
		if !isDP && !isMethod {
			t.Errorf("button %q maps to %q, which is neither a dpCommand nor a Method", b.id, b.cmd)
		}
	}
}

// The select options must be the same strings setIndex resolves back to enum
// values, or a valid pick from the dashboard is rejected.
func TestSelectOptionsRoundTrip(t *testing.T) {
	h := testBridge()
	for name, field := range map[string][]string{
		"water_level": mopNames,
		"clean_type":  typeNames,
	} {
		c := h.components()[name].(map[string]any)
		opts, ok := c["options"].([]string)
		if !ok {
			t.Fatalf("%s has no options list", name)
		}
		if len(opts) != len(field) {
			t.Errorf("%s options %v do not match the enum %v", name, opts, field)
		}
		for i, o := range opts {
			if o != field[i] {
				t.Errorf("%s option %d = %q, enum has %q", name, i, o, field[i])
			}
		}
	}
}

// clean_spot was mapped to cruise, method 20, which is not spot clean.
// SPOT_CLEAN has no encoder, so the feature must not be declared at all.
func TestNoUnbackedFeatures(t *testing.T) {
	for _, f := range haFeatures {
		if f == "clean_spot" {
			t.Error("clean_spot declared, but SPOT_CLEAN has no param encoder")
		}
	}
}

// Every button must be reachable by some path: a ModeCtrl method, the bus, or
// the data point the bridge lends its connection to. Dust is deliberately on
// the data point - the bus action empties the bin and skips the hair cut.
func TestStationButtonsAreReachable(t *testing.T) {
	for _, b := range haButtons {
		if _, ok := client.Methods[b.cmd]; ok {
			continue
		}
		_, onBus := busStationActions[b.cmd]
		_, onDP := dp.StationActions[b.cmd]
		if !onBus && !onDP {
			t.Errorf("button %q runs %q, which has neither a bus nor a data-point encoding",
				b.id, b.cmd)
		}
	}
}

// A configured but not-yet-connected internal broker must still read as a data
// point path. The HA bridge lends its own connection only when HasBus is false;
// guarding that on Ready instead lost this window and let the bridge replace a
// live internal broker, routing DPs to a broker cloud_cli no longer listens on.
func TestConfiguredBrokerCountsAsAPathBeforeConnect(t *testing.T) {
	cloud := cloud.NewCloud("AMP00X0E00000000", "eufy_home")
	broker, err := mqtt.NewBrokerBus("127.0.0.1:0", "", "", true, cloud.Handle)
	if err != nil {
		t.Fatal(err)
	}
	cloud.SetBus(broker)
	if cloud.Ready() {
		t.Fatal("broker reports Connected with no client; the lend race cannot be reproduced")
	}
	if !cloud.HasBus() {
		t.Error("HasBus() is false for a configured internal broker; the HA bridge would steal the path")
	}
}

// The data point x10-ctl sends for dust must stay byte-identical to the app's,
// which was captured on 2026-08-07 as f2{f3:1}.
func TestDustDataPointMatchesTheApp(t *testing.T) {
	v, err := dp.StationAction("dust")
	if err != nil {
		t.Fatal(err)
	}
	if got := unwrapDP(t, v); got != "12021801" {
		t.Errorf("dust DP = %s, want 12021801 as the app sends", got)
	}
	if _, onBus := busStationActions["dust"]; onBus {
		t.Error("dust is back on the bus path, which skips the hair cut")
	}
}

// Room selection is named in the done-when, so it must be native rather than
// only reachable through send_command.
func TestRoomSelectionIsNative(t *testing.T) {
	v := testBridge().components()["vacuum"].(map[string]any)
	topic, ok := v["clean_segments_command_topic"].(string)
	if !ok {
		t.Fatal("no clean_segments_command_topic; rooms are send_command only")
	}
	if !strings.HasPrefix(topic, testBridge().base+"/cmd/") {
		t.Errorf("clean_segments topic %q is not subscribed", topic)
	}
}

func TestCleanSegmentsParsing(t *testing.T) {
	h := testBridge()
	for _, bad := range []string{"", "{}", "[]", `["kitchen"]`, `[true]`} {
		if err := h.cleanSegments(t.Context(), bad); err == nil {
			t.Errorf("cleanSegments(%q) was accepted", bad)
		}
	}
}

// HA publishes online to homeassistant/status when it restarts, and its entity
// registry may be empty. If we are not subscribed, nothing refills it.
func TestSubscribesToHAStatus(t *testing.T) {
	h := testBridge()
	bus := &mqtt.ClientBus{}
	h.attach(bus)
	var found bool
	for _, tp := range bus.Topics {
		if tp == haPrefix+"/status" {
			found = true
		}
	}
	if !found {
		t.Errorf("not subscribed to %s/status; topics = %v", haPrefix, bus.Topics)
	}
}

// attach must not clobber credentials the caller already set.
func TestAttachKeepsCredentials(t *testing.T) {
	bus := &mqtt.ClientBus{}
	bus.Opts.Username, bus.Opts.Password = "ha", "pw"
	testBridge().attach(bus)
	if bus.Opts.Username != "ha" || bus.Opts.Password != "pw" {
		t.Error("attach wiped the credentials")
	}
	if bus.Opts.Will == nil {
		t.Error("attach did not set the will")
	}
}

// AUTO_CLEAN was fired on hardware and the robot did not move, so start must
// not route to it.
func TestStartDoesNotUseAutoClean(t *testing.T) {
	src, err := os.ReadFile("ha.go")
	if err != nil {
		t.Fatal(err)
	}
	body := string(src)
	i := strings.Index(body, `case "start", "clean", "turn_on":`)
	if i < 0 {
		t.Fatal("start case not found")
	}
	if strings.Contains(body[i:i+300], `"autoclean"`) {
		t.Error("start routes to autoclean, which does nothing on this robot")
	}
}

// The app ends a run with GOHOME. Method 12 is exit-remote-control and does
// nothing to a clean, which cost most of a day to establish.
func TestStopIsGohomeNotStopTask(t *testing.T) {
	src, err := os.ReadFile("ha.go")
	if err != nil {
		t.Fatal(err)
	}
	body := string(src)
	i := strings.Index(body, `case "stop", "turn_off":`)
	if i < 0 {
		t.Fatal("stop case not found")
	}
	seg := body[i : i+300]
	if !strings.Contains(seg, `"gohome"`) {
		t.Error("stop does not route to gohome")
	}
	if strings.Contains(seg, `h.method(ctx, "stop")`) {
		t.Error("stop still routes to method 12, which does not stop a clean")
	}
}

// mds_pub succeeds for pause and the robot ignores it, so trying the local path
// first means never reaching the one that works.
func TestPauseSkipsTheLocalPath(t *testing.T) {
	if !dataPointOnly["pause"] || !dataPointOnly["resume"] {
		t.Error("pause and resume must be data-point only")
	}
	for _, n := range []string{"gohome", "autoclean", "roomsclean"} {
		if dataPointOnly[n] {
			t.Errorf("%s works on the bus and must not be forced onto the data point", n)
		}
	}
}

// unwrapDP reverses dp.DPValue: base64, then the varint length prefix.
func unwrapDP(t *testing.T, v string) string {
	t.Helper()
	raw, err := base64.StdEncoding.DecodeString(v)
	if err != nil {
		t.Fatalf("not base64: %v", err)
	}
	n, off, ok := pb.ReadVarint(raw, 0)
	if !ok {
		t.Fatal("no length prefix")
	}
	if off+int(n) != len(raw) {
		t.Fatalf("length prefix says %d, got %d bytes", n, len(raw)-off)
	}
	return hex.EncodeToString(raw[off:])
}
