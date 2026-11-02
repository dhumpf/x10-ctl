package dp

// The encoder tests pin bytes that were captured from the vendor, not bytes
// this program invented. Where a command was confirmed on hardware the hex is
// quoted from the document that recorded it, so a refactor that changes the
// wire format fails here rather than on the robot.

import (
	"encoding/base64"
	"encoding/hex"
	"github.com/dhumpf/x10-ctl/internal/pb"
	"testing"
)

// unwrapDP reverses dpValue: base64, then the varint length prefix.
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

// The two dry configurations quoted in eufy-cloud-protocol.md, both sent
// through the real path. The empty Duration on SHORT is the vendor's shape and
// is deliberately reproduced - see dryConfig.
func TestDryConfigMatchesCapture(t *testing.T) {
	for _, tc := range []struct{ mode, duration, want string }{
		{"standard", "long", "0a081206080112020802"},
		{"standard", "short", "0a06120408011200"},
	} {
		got, err := DryConfig(tc.mode, tc.duration)
		if err != nil {
			t.Fatalf("%s/%s: %v", tc.mode, tc.duration, err)
		}
		if h := unwrapDP(t, got); h != tc.want {
			t.Errorf("dryConfig(%s,%s) = %s, want %s", tc.mode, tc.duration, h, tc.want)
		}
	}
}

func TestDryConfigRejectsUnknownLevels(t *testing.T) {
	// Both enums are 0-2. The app shows 1/2/3 hours, which are levels, not
	// values, and "3" has to stay an error rather than becoming a fourth level.
	if _, err := DryConfig("standard", "3"); err == nil {
		t.Error("duration \"3\" accepted; both enums are 0-2")
	}
	if _, err := DryConfig("turbo", "short"); err == nil {
		t.Error("unknown dry mode accepted")
	}
}

// ManualActionCmd field numbers, from the oneof. dust was captured
// byte-identical to the app's own command.
func TestStationActionWire(t *testing.T) {
	for _, tc := range []struct{ name, want string }{
		{"dry", "12021001"},  // 2{2:1}
		{"dust", "12021801"}, // 2{3:1}
	} {
		got, err := StationAction(tc.name)
		if err != nil {
			t.Fatalf("%s: %v", tc.name, err)
		}
		if h := unwrapDP(t, got); h != tc.want {
			t.Errorf("stationAction(%s) = %s, want %s", tc.name, h, tc.want)
		}
	}
	if _, err := StationAction("levitate"); err == nil {
		t.Error("unknown station action accepted")
	}
}

func TestDecodeWorkStatus(t *testing.T) {
	// work_status field 2 carries the state; 3 is docked.
	v := DPValue(pb.FieldVarint(2, 3))
	if got := DecodeDP(DPWorkStatus, v); got != "docked" {
		t.Errorf("work_status = %q, want %q", got, "docked")
	}
	// An unseen value is reported, not guessed at.
	v = DPValue(pb.FieldVarint(2, 99))
	if got := DecodeDP(DPWorkStatus, v); got != "state 99" {
		t.Errorf("unknown state = %q, want %q", got, "state 99")
	}
	// Plain scalars pass through untouched.
	if got := DecodeDP(DPBatLevel, float64(88)); got != "88" {
		t.Errorf("bat_level = %q, want 88", got)
	}
}

// --- MQTT wire

// --- end to end, both halves of the default mode
