package app

import (
	"encoding/hex"
	"os"
	"path/filepath"
	"testing"
	"time"
)

// The robot's own user_config.yaml, trimmed to the keys these tests touch.
// Values are the ones read off the device, including the dust bag being the one
// consumable whose pair has moved.
const sampleUserConfig = `user_total_clean_area: 163
last_change_dust_bag_total_clean_area: 110
children_lock_switch: false
clean_type: 2
voice_volume: 10
undisturbed_switch: true
undisturbed_start_hour: 22
undisturbed_start_minute: 0
undisturbed_end_hour: 8
undisturbed_end_minute: 30
multi_map_switch: true
ai_see_switch: true
water_level_sw: true
suggest_restricted_zone_sw: true
dust_full_remind: 36000
live_photo_sw: false
follow_mode_sw: true
side_brush_extend_sw: true
enable_dirty_area_focus_clean_sw: false
poop_sw: true
pet_mode_sw: false
side_brush_run_time: 13380
rolling_brush_run_time: 13380
mop_run_time: 2460
dustbag_run_time: 1549
maintenance_side_brush_duration: 13380
maintenance_rolling_brush_duration: 13380
maintenance_mop_duration: 2460
maintenance_dustbag_duration: 1297
user_total_clean_duration: 9600
user_total_clean_count: 6
last_clean_time: 1785427795
`

func writeSampleConfig(t *testing.T) string {
	t.Helper()
	p := filepath.Join(t.TempDir(), "user_config.yaml")
	if err := os.WriteFile(p, []byte(sampleUserConfig), 0o600); err != nil {
		t.Fatal(err)
	}
	return p
}

// TestEncodeUndisturbedMatchesRecordedFrame pins the encoder to the frame that
// was confirmed on hardware: on, 22:00 to 08:30, the window this robot is set
// to.
//
// The request wrapper - field 2 holding field 1 holding the window - is the
// whole point of this test. Without it the frame still transmits and the robot
// still accepts it, and the setting comes back as off with all four times zero.
func TestEncodeUndisturbedMatchesRecordedFrame(t *testing.T) {
	got := hex.EncodeToString(encodeUndisturbed(Undisturbed{
		Enabled: true,
		Begin:   TimePoint{Hour: 22},
		End:     TimePoint{Hour: 8, Minute: 30},
	}))
	want := "12100a0e0a020801120208161a040808101e"
	if got != want {
		t.Fatalf("frame\n got %s\nwant %s", got, want)
	}
}

// A switch that is off is an empty submessage, not an absent one: proto3 drops
// the false, but the field itself has to be there or the robot is being told
// nothing about it rather than being told to turn it off.
func TestEncodeUndisturbedOffKeepsTheSwitchField(t *testing.T) {
	got := hex.EncodeToString(encodeUndisturbed(Undisturbed{
		Begin: TimePoint{Hour: 22},
		End:   TimePoint{Hour: 8, Minute: 30},
	}))
	want := "120e0a0c0a00120208161a040808101e"
	if got != want {
		t.Fatalf("frame\n got %s\nwant %s", got, want)
	}
}

// The OFF frame is the one record the live StationSetting channel carried
// before the robot rebooted, and it was the write that turned auto-empty off.
// Reproducing it byte for byte is what makes the ON frame trustworthy.
func TestEncodeCollectDustMatchesRecordedFrame(t *testing.T) {
	if got := hex.EncodeToString(encodeCollectDust(false)); got != "12060a0432020a00" {
		t.Fatalf("off frame\n got %s\nwant 12060a0432020a00", got)
	}
	// On differs only by the Switch carrying its true.
	if got := hex.EncodeToString(encodeCollectDust(true)); got != "12080a0632040a020801" {
		t.Fatalf("on frame\n got %s\nwant 12080a0632040a020801", got)
	}
}

// By task with task 1 is "empty after every clean". Mode 0 is proto3's default
// and drops out, so the Mode submessage carries the task count alone.
func TestEncodeCollectDustFullEveryClean(t *testing.T) {
	const want = "120c0a0a32080a02080112021001"
	if got := hex.EncodeToString(encodeCollectDustFull(true, 0, 1, 0)); got != want {
		t.Fatalf("frame\n got %s\nwant %s", got, want)
	}
	// The switch half must stay identical to the confirmed frame.
	if got := hex.EncodeToString(encodeCollectDustFull(true, 0, 0, 0)); got != "120a0a0832060a0208011200" {
		t.Fatalf("bare frame\n got %s", got)
	}
}

func TestUndisturbedActiveWrapsMidnight(t *testing.T) {
	u := Undisturbed{Enabled: true, Begin: TimePoint{Hour: 22}, End: TimePoint{Hour: 8, Minute: 30}}
	at := func(h, m int) time.Time { return time.Date(2026, 7, 31, h, m, 0, 0, time.UTC) }
	for _, c := range []struct {
		h, m int
		want bool
	}{
		{21, 59, false}, {22, 0, true}, {23, 30, true},
		{0, 1, true}, {8, 29, true}, {8, 30, false}, {12, 0, false},
	} {
		if got := u.Active(at(c.h, c.m)); got != c.want {
			t.Errorf("%02d:%02d active=%v want %v", c.h, c.m, got, c.want)
		}
	}
	off := u
	off.Enabled = false
	if off.Active(at(23, 0)) {
		t.Error("a disabled window is never active")
	}
	empty := Undisturbed{Enabled: true, Begin: TimePoint{Hour: 3}, End: TimePoint{Hour: 3}}
	if empty.Active(at(3, 0)) {
		t.Error("equal endpoints are a zero-length window, not all day")
	}
}

func TestReadUndisturbedFromConfig(t *testing.T) {
	u, err := readUndisturbedFrom(writeSampleConfig(t))
	if err != nil {
		t.Fatal(err)
	}
	if !u.Enabled || u.Begin.Hour != 22 || u.Begin.Minute != 0 || u.End.Hour != 8 || u.End.Minute != 30 {
		t.Fatalf("got %+v", u)
	}
	if u.Begin.String() != "22:00" || u.End.String() != "08:30" {
		t.Fatalf("format: %s - %s", u.Begin, u.End)
	}
}

func TestEncodeUniSettingsFieldNumbers(t *testing.T) {
	yes, no := true, false
	got, err := encodeUniSettings(&UniSettings{Switches: map[string]*bool{"children_lock": &yes}})
	if err != nil {
		t.Fatal(err)
	}
	// field 2 { field 1 { field 1 = 1 } }
	if h := hex.EncodeToString(got); h != "12040a020801" {
		t.Fatalf("children_lock on: got %s", h)
	}
	got, err = encodeUniSettings(&UniSettings{Switches: map[string]*bool{"pet_mode_sw": &no}})
	if err != nil {
		t.Fatal(err)
	}
	// pet_mode_sw is REQUEST field 14 (0x72), not response field 15.
	if h := hex.EncodeToString(got); h != "12027200" {
		t.Fatalf("pet_mode_sw off: got %s", h)
	}
}

func TestEncodeUniSettingsRejectsNonsense(t *testing.T) {
	yes := true
	if _, err := encodeUniSettings(&UniSettings{Switches: map[string]*bool{"no_such_switch": &yes}}); err == nil {
		t.Error("unknown switch should be rejected, not silently dropped")
	}
	if _, err := encodeUniSettings(&UniSettings{}); err == nil {
		t.Error("an empty write should be rejected")
	}
	bad := 9
	if _, err := encodeUniSettings(&UniSettings{HighDetection: &bad}); err == nil {
		t.Error("high_detection out of range should be rejected")
	}
}

// Every switch must have a field number that exists once. A duplicate would
// make one switch silently write another.
func TestUniSwitchFieldNumbersUnique(t *testing.T) {
	seen := map[uint64]string{}
	for _, s := range uniSwitches {
		if s.Num == 0 || s.Num == 5 || s.Num == 6 {
			t.Errorf("%s: field %d is not a switch field", s.Name, s.Num)
		}
		if prev, dup := seen[s.Num]; dup {
			t.Errorf("field %d is used by both %s and %s", s.Num, prev, s.Name)
		}
		seen[s.Num] = s.Name
	}
	if _, taken := seen[dustFullRemindField]; taken {
		t.Error("dust_full_remind's field is also claimed by a switch")
	}
	if _, taken := seen[highDetectionField]; taken {
		t.Error("high_detection's field is also claimed by a switch")
	}
}

// Mapped and YAML have to agree. Mapped means "we know which key reads this
// back", not "the write works" - six of these were wrongly marked as ignored
// until the eMMC backup showed two of them had changed keys nobody was watching.
func TestUniSwitchMappedMatchesKey(t *testing.T) {
	unmapped := 0
	for _, s := range uniSwitches {
		if (s.YAML != "") != s.Mapped {
			t.Errorf("%s: yaml=%q mapped=%v", s.Name, s.YAML, s.Mapped)
		}
		if !s.Mapped {
			unmapped++
		}
	}
	if unmapped != 6 {
		t.Errorf("six switches have no known readback key, table says %d", unmapped)
	}
}

func TestReadUniSettingsFromConfig(t *testing.T) {
	got := readUniSettingsFrom(writeSampleConfig(t))
	if got["children_lock"] != false {
		t.Errorf("children_lock: %v", got["children_lock"])
	}
	if got["pet_mode_sw"] != false || got["poop_avoidance_sw"] != true {
		t.Errorf("pet %v poop %v", got["pet_mode_sw"], got["poop_avoidance_sw"])
	}
	if got["dust_full_remind"] != 36000 {
		t.Errorf("dust_full_remind: %v", got["dust_full_remind"])
	}
	// A switch with no known key reads as null rather than as false, which
	// would be a claim the file does not make.
	if v, ok := got["deep_mop_corner_sw"]; !ok || v != nil {
		t.Errorf("deep_mop_corner_sw should be nil, got %v", v)
	}
}

func TestReadConsumables(t *testing.T) {
	path := writeSampleConfig(t)
	parts := readConsumablesFrom(path)
	byName := map[string]consumableState{}
	for _, p := range parts {
		byName[p.Name] = p
	}
	bag := byName["dust bag"]
	if bag.TotalMin == nil || bag.LeftMin == nil || *bag.TotalMin != 1549 || *bag.LeftMin != 1297 {
		t.Fatalf("dust bag: %+v", bag)
	}
	if bag.UsedMin == nil || *bag.UsedMin != 252 {
		t.Fatalf("dust bag used: %+v", bag.UsedMin)
	}
	if bag.Percent == nil || *bag.Percent != 83 {
		t.Fatalf("dust bag percent: %+v", bag.Percent)
	}
	brush := byName["side brush"]
	if brush.Percent == nil || *brush.Percent != 100 {
		t.Fatalf("an unused part is 100%%, got %+v", brush.Percent)
	}
	if brush.Attention {
		t.Error("a full part should not ask for attention")
	}

	totals := cleaningTotalsFrom(path)
	if totals["area_m2"] != 163 || totals["cleans"] != 6 {
		t.Fatalf("totals: %v", totals)
	}
	if totals["area_m2_since_bag_change"] != 53 {
		t.Fatalf("area since bag change: %v", totals["area_m2_since_bag_change"])
	}
}

// Two keys that merely look like a pair must not produce a percentage. The
// scraper is the case: both keys exist and read plausible numbers, and pairing
// them claimed 69% of a part that has never been used.
func TestUnpairedConsumableHasNoPercentage(t *testing.T) {
	p := filepath.Join(t.TempDir(), "user_config.yaml")
	body := "scrape_run_time: 13380\nmaintenance_mid_drum_mop_wiper_duration: 9240\n"
	if err := os.WriteFile(p, []byte(body), 0o600); err != nil {
		t.Fatal(err)
	}
	for _, c := range readConsumablesFrom(p) {
		if c.Name != "scraper" {
			continue
		}
		if c.TotalMin == nil || c.LeftMin == nil {
			t.Fatal("both numbers should still be reported")
		}
		if c.Percent != nil || c.UsedMin != nil {
			t.Fatalf("unpaired keys must not produce an age: %+v", c)
		}
		if len(c.Notes) == 0 {
			t.Fatal("an unpaired part should say why there is no percentage")
		}
	}
}

// A pair the other way round must be reported, not turned into a negative age.
func TestConsumableReversedPairIsFlagged(t *testing.T) {
	p := filepath.Join(t.TempDir(), "user_config.yaml")
	body := "mop_run_time: 100\nmaintenance_mop_duration: 250\n"
	if err := os.WriteFile(p, []byte(body), 0o600); err != nil {
		t.Fatal(err)
	}
	for _, c := range readConsumablesFrom(p) {
		if c.Name != "mop" {
			continue
		}
		if c.UsedMin != nil || c.Percent != nil {
			t.Fatalf("should not compute an age from a reversed pair: %+v", c)
		}
		if len(c.Notes) == 0 {
			t.Fatal("a reversed pair should say so")
		}
	}
}
