package cloud

import (
	"encoding/hex"
	"strings"
	"testing"
)

// Sweep-only is enum 0, and a scalar enum at 0 is dropped by proto3. The only
// way to say it is a present but empty submessage, which is what the app sends.
func TestSweepOnlyIsAnEmptySubmessage(t *testing.T) {
	if h := hex.EncodeToString(enumSub(1, 0)); h != "0a00" {
		t.Errorf("clean_type 0 = %s, want 0a00 as the app sends", h)
	}
	if h := hex.EncodeToString(enumSub(1, 2)); h != "0a020802" {
		t.Errorf("clean_type 2 = %s, want 0a020802", h)
	}
}

// The frame shape the app sent: f1=5, f2=seq, f3=map, f10{ ... f3 per room }.
func TestRoomParamsShape(t *testing.T) {
	body := roomParamsDP(77, []RoomParam{{ID: 2, CleanType: 0, Suction: 1, CleanTimes: 1}}, 105)
	h := hex.EncodeToString(body)
	for name, frag := range map[string]string{
		"f1 = 5":           "0805",
		"f2 = seq 105":     "1069",
		"f3 = map 77":      "184d",
		"f10 present":      "52",
		"room entry in f3": "1a",
		"room id 2":        "0802",
		"sweep_only":       "0a00",
		"suction 1":        "1202" + "0801",
	} {
		if !strings.Contains(h, frag) {
			t.Errorf("%s: missing %s in %s", name, frag, h)
		}
	}
}

// The per-room block writes only the fields whose meaning is established.
// f3, f4 and f6 were in the capture and are deliberately left out: an unknown
// field guessed wrong is what turns a vacuum run into a mop run.
func TestRoomParamsWritesOnlyKnownFields(t *testing.T) {
	// One room, everything at its default, so the block is as small as it gets.
	body := roomParamsDP(77, []RoomParam{{ID: 2}}, 1)
	h := hex.EncodeToString(body)
	// The room entry is f1{f1{ f1=id, f2=params }}; the params are the tail.
	i := strings.Index(h, "1a")
	i = strings.Index(h, "0802")
	if i < 0 {
		t.Fatal("no room entry")
	}
	params := h[i+4:]
	// clean_type, suction, water, intensity - all four at enum 0, so all four
	// are empty submessages. f3.f2, f3.f3 and f6 stay unwritten.
	if !strings.HasPrefix(params, "12080a0012001a002200") {
		t.Errorf("room params = %s, want the four known fields only", params)
	}
	if strings.Contains(params, "3200") {
		t.Errorf("wrote f6, which is still unknown: %s", params)
	}
}

// Water is f3.f1 and intensity is f4.f1, each mapped from three captured
// values. Water low and intensity medium are enum 0 and vanish.
func TestWaterAndIntensityFields(t *testing.T) {
	for _, tc := range []struct {
		water, intensity int
		wantWater        string
		wantIntensity    string
	}{
		{0, 0, "1a00", "2200"},
		{1, 1, "1a020801", "22020801"},
		{2, 2, "1a020802", "22020802"},
	} {
		h := hex.EncodeToString(roomParamsDP(77, []RoomParam{
			{ID: 2, Water: tc.water, Intensity: tc.intensity}}, 1))
		if !strings.Contains(h, tc.wantWater) {
			t.Errorf("water %d: want %s in %s", tc.water, tc.wantWater, h)
		}
		if !strings.Contains(h, tc.wantIntensity) {
			t.Errorf("intensity %d: want %s in %s", tc.intensity, tc.wantIntensity, h)
		}
	}
}
