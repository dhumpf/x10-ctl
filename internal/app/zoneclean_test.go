package app

import (
	"encoding/hex"
	"github.com/dhumpf/x10-ctl/internal/dp"
	"github.com/dhumpf/x10-ctl/internal/pb"
	"testing"
)

// The frame the vendor app sent on 2026-08-07 for a zone clean, minus the
// method and seq. Anything that changes these bytes changes what the robot does.
//
// Point is sint32, so the wire carries ZIGZAG world centimetres. Reading those
// bytes as plain varints is what produced this file's old both-axes mirror; the
// capture never had one.
func TestZonesParamIsZigzagWorldCentimetres(t *testing.T) {
	// Living Room, layer cm x 310-815, y 1140-1545, against the map origin
	// (-9.475, -13.525) m. World cm: x -637..-132, y -212..+192.
	got := zonesParam([]ZoneRect{{X1: 310, Y1: 1140, X2: 815, Y2: 1545}}, 77, 1, -947, -1352)
	h := hex.EncodeToString(got)
	for _, c := range []struct {
		name string
		frag string
	}{
		{"x1 -637", hex.EncodeToString(pb.FieldSint(1, -637))},
		{"x2 -132", hex.EncodeToString(pb.FieldSint(1, -132))},
		{"y1 -212", hex.EncodeToString(pb.FieldSint(2, -212))},
		{"y2 193", hex.EncodeToString(pb.FieldSint(2, 193))},
	} {
		if !contains(h, c.frag) {
			t.Errorf("missing %s (%s) in %s", c.name, c.frag, h)
		}
	}
	// Untagged: the body starts with the first Zone, field 1. ModeCtrlParam is
	// what puts it in field 5, and hardcoding 4 there is what broke this.
	if h[:2] != "0a" {
		t.Errorf("zones body must be untagged, got %s", h[:2])
	}
}

// The dock is the one point readable in two frames at once, so it pins the
// origin shift without trusting any earlier reading.
func TestZonesParamAgreesWithTheDockPose(t *testing.T) {
	// Dock: layer (921, 1350), origin (-947.5, -1352.5) cm, /api/pose (-26.7, -2.9).
	got := zonesParam([]ZoneRect{{X1: 921, Y1: 1350, X2: 921, Y2: 1350}}, 77, 1, -947, -1352)
	h := hex.EncodeToString(got)
	if !contains(h, hex.EncodeToString(pb.FieldSint(1, -26))) {
		t.Errorf("dock x is not -26 cm from the origin: %s", h)
	}
	if !contains(h, hex.EncodeToString(pb.FieldSint(2, -2))) {
		t.Errorf("dock y is not -2 cm from the origin: %s", h)
	}
}

// Spot carries no coordinate, and no tag of its own either.
func TestSpotParamIsUntagged(t *testing.T) {
	if h := hex.EncodeToString(spotParam()); h != "0801" {
		t.Errorf("spotParam = %s, want 0801", h)
	}
}

// The oneof field is the method's, not a constant. Sending zones as f4{f5{}}
// and spot as f4{f6{}} is what the robot dropped without a word.
func TestParamFieldPerMethod(t *testing.T) {
	for _, c := range []struct {
		method, field uint64
	}{
		{dp.MethodAutoClean, 3}, {dp.MethodRoomClean, 4},
		{dp.MethodZonesClean, 5}, {dp.MethodSpotClean, 6},
	} {
		if got := dp.ParamField(c.method); got != c.field {
			t.Errorf("method %d param field = %d, want %d", c.method, got, c.field)
		}
	}
}

func contains(h, frag string) bool {
	for i := 0; i+len(frag) <= len(h); i++ {
		if h[i:i+len(frag)] == frag {
			return true
		}
	}
	return false
}
