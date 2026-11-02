package app

// The switch block - child lock, pet mode, carpet handling, obstacle
// avoidance, dust-bag reminder. The app calls it "unisetting" and sends it as
// data point 176; this writes eros.proto.msg.app.Unisetting on the bus instead,
// for the same reason as everything else here.
//
//	Unisetting        { Header header = 1; UnisettingRequest req = 2 }
//	UnisettingRequest { Switch children_lock = 1; ... HighDetection = 21 }
//	Switch            { bool value = 1 }
//	Numerical         { uint32 value = 1 }
//
// Request and response number their fields DIFFERENTLY, and that is the trap in
// this message. UnisettingRequest has multi_map_options at 5 and wifi_setting
// at 6, which the response does not carry at all, so every switch after
// ai_see sits two lower on the way back than it did on the way out:
//
//	                        request  response
//	water_level_sw               7       5
//	dust_full_remind            10       8
//	pet_mode_sw                 14      15
//
// The table below is the REQUEST numbering, because this file only writes.
//
// App labels, where they are known, because the switch names here are not what
// a user sees: field 12 is "Smart Track Cleaning", 13 and 14 are both set by
// the single "Pet Mode" toggle. The gloss on 12 used to read "follow a person",
// which the app's own wording does not support.
//
// Five fields verified against live app writes on 2026-08-06: 4, 7, 12, 13 and 14,
// all where this table says. Field 7 is the decisive one - it is a switch whose
// request and response numbers differ, 7 out and 5 back, and the app sent 7. So
// the app writes request numbering and this table is the right one to encode
// against.
//
// Enabling Pet Mode sent DP
// 176 as `f13 { f1: 1 } f14 { f1: 1 }` and user_config then read poop_sw true
// and pet_mode_sw true - field 13 and field 14 exactly as listed. Two fields is
// not the whole table, but it is the first time any of it has been checked
// against the vendor app rather than derived, and both landed.
// Anything reading a response has to renumber, and the recorded DP 176 values
// (`8{1:36000}` for the dust-bag reminder, `11:58` for signal strength) are
// response-numbered - 8 is dust_full_remind there and deep_mop_corner here.
//
// Reads do not go near either numbering: they come from user_config.yaml, which
// cloud_cli rewrites whenever a setting changes. Twelve of the twenty-one
// switches have a key there and are reported; the rest are write-only until
// something is seen to persist them, and are reported as null rather than
// guessed at.

import (
	"context"
	"fmt"
	"github.com/dhumpf/x10-ctl/internal/pb"
	"net/http"
	"sort"
	"time"
)

const mdsUnisetting = "eros.proto.msg.app.Unisetting"

// uniSwitch is one entry in the block.
//
// yaml is the user_config.yaml key this switch reads back from, empty when no
// key has been identified for it.
//
// Empty does NOT mean the write is ignored. That was claimed here after setting
// six of them and diffing the file a second later, and it was wrong: comparing
// against the 2026-07-29 eMMC backup showed two keys had in fact flipped -
// enable_dirty_area_focus_clean_sw and enable_stain_cruise_clean_mode. cloud_cli
// writes the file on its own schedule, so a diff taken a second after the
// publish sees nothing. Anything checked that way needs minutes, or a known-good
// baseline to compare against.
type uniSwitch struct {
	Name   string `json:"name"`
	Num    uint64 `json:"field"`
	YAML   string `json:"-"`
	Desc   string `json:"desc"`
	Mapped bool   `json:"mapped"`
}

// uniSwitches is the writable block, in proto field order.
//
// Fields 5 and 6 are absent on purpose: MultiMapOptions carries a map-id
// retention list and WifiSetting carries an SSID deletion list, neither of
// which is a switch, and both of which delete something when written wrong.
// Field 21 is HighDetection, an enum rather than a switch, and is handled
// separately below.
var uniSwitches = []uniSwitch{
	{"children_lock", 1, "children_lock_switch", "keypad lock on the robot", true},
	{"cruise_continue_sw", 2, "", "carry on cruising after a charge", false},
	{"multi_map_sw", 3, "multi_map_switch", "keep more than one map", true},
	{"ai_see", 4, "ai_see_switch", "obstacle recognition from the camera", true},
	{"water_level_sw", 7, "water_level_sw", "water tank level sensing", true},
	{"suggest_restricted_zone_sw", 8, "suggest_restricted_zone_sw", "suggest keep-out zones", true},
	{"deep_mop_corner_sw", 9, "", "extra corner mopping", false},
	{"live_photo_sw", 11, "live_photo_sw", "upload photos of what it finds", true},
	{"smart_follow_sw", 12, "follow_mode_sw", "smart track cleaning", true},
	{"poop_avoidance_sw", 13, "poop_sw", "pet waste avoidance", true},
	{"pet_mode_sw", 14, "pet_mode_sw", "pet mode", true},
	{"side_brush_extend_sw", 15, "side_brush_extend_sw", "extend the side brush into corners", true},
	{"first_clean_carpet_sw", 16, "", "do carpets first", false},
	{"dirty_area_focus_clean_sw", 17, "enable_dirty_area_focus_clean_sw", "go back over dirty patches", true},
	{"dirty_area_second_clean_sw", 18, "", "second pass on dirty patches", false},
	{"gesture_point_clean_sw", 19, "", "point-to-clean by gesture", false},
	{"dirty_inspection_sw", 20, "", "check how dirty the floor is", false},
}

// dustFullRemind is a Numerical, not a Switch: the value is the dust-bag
// reminder threshold, seen as 36000 on this robot and as `8{1:36000}` on the
// data point.
const dustFullRemindField = 10

// highDetectionNames index by wire value, so a lookup is also the validation.
var highDetectionNames = []string{"safe", "medium", "free"}

const highDetectionField = 21

// UniSettings is a partial write. Every field is a pointer so an absent one
// means "leave it alone": the robot merges what it is sent, and modelling an
// unset switch as false would turn every write into a write of all of them.
type UniSettings struct {
	Switches       map[string]*bool `json:"switches"`
	DustFullRemind *int             `json:"dust_full_remind"`
	HighDetection  *int             `json:"high_detection"`
}

func uniByName(name string) (uniSwitch, bool) {
	for _, s := range uniSwitches {
		if s.Name == name {
			return s, true
		}
	}
	return uniSwitch{}, false
}

// readUniSettings reports what user_config.yaml persists. A switch with no
// known key reads as null.
func readUniSettings() map[string]any { return readUniSettingsFrom(userConfig) }

func readUniSettingsFrom(path string) map[string]any {
	out := map[string]any{}
	for _, s := range uniSwitches {
		if s.YAML == "" {
			out[s.Name] = nil
			continue
		}
		v, ok := scanKey(path, s.YAML)
		if !ok {
			out[s.Name] = nil
			continue
		}
		out[s.Name] = yamlBool(v)
	}
	if v, err := scanInt(path, "dust_full_remind"); err == nil {
		out["dust_full_remind"] = v
	} else {
		out["dust_full_remind"] = nil
	}
	return out
}

// encodeUniSettings builds the Unisetting payload for the fields that were set.
func encodeUniSettings(u *UniSettings) ([]byte, error) {
	var req []byte
	names := make([]string, 0, len(u.Switches))
	for name := range u.Switches {
		names = append(names, name)
	}
	// Sorted so the same request always produces the same bytes, which is what
	// makes a recorded frame worth comparing against.
	sort.Strings(names)
	for _, name := range names {
		v := u.Switches[name]
		if v == nil {
			continue
		}
		sw, ok := uniByName(name)
		if !ok {
			return nil, fmt.Errorf("unknown switch %q", name)
		}
		var inner []byte
		if *v {
			inner = pb.FieldVarint(1, 1)
		}
		req = append(req, pb.FieldBytes(sw.Num, inner)...)
	}
	if u.DustFullRemind != nil {
		if *u.DustFullRemind < 0 {
			return nil, fmt.Errorf("dust_full_remind must not be negative")
		}
		var inner []byte
		if *u.DustFullRemind != 0 {
			inner = pb.FieldVarint(1, uint64(*u.DustFullRemind))
		}
		req = append(req, pb.FieldBytes(dustFullRemindField, inner)...)
	}
	if u.HighDetection != nil {
		if *u.HighDetection < 0 || *u.HighDetection >= len(highDetectionNames) {
			return nil, fmt.Errorf("high_detection must be 0..%d", len(highDetectionNames)-1)
		}
		var inner []byte
		if *u.HighDetection != 0 {
			inner = pb.FieldVarint(1, uint64(*u.HighDetection))
		}
		req = append(req, pb.FieldBytes(highDetectionField, inner)...)
	}
	if len(req) == 0 {
		return nil, fmt.Errorf("nothing to set")
	}
	return pb.FieldBytes(2, req), nil
}

// SetUniSettings publishes a partial switch block onto the bus.
func (l *Local) SetUniSettings(ctx context.Context, u *UniSettings) error {
	payload, err := encodeUniSettings(u)
	if err != nil {
		return err
	}
	return l.mdsPublish(ctx, mdsUnisetting, payload)
}

func (s *server) handleUniSettings(w http.ResponseWriter, r *http.Request) {
	respond := func(extra map[string]any) {
		out := map[string]any{
			"ok":       true,
			"settings": readUniSettings(),
			"fields":   uniSwitches,
			"names":    map[string]any{"high_detection": highDetectionNames},
		}
		for k, v := range extra {
			out[k] = v
		}
		writeJSON(w, http.StatusOK, out)
	}

	switch r.Method {
	case http.MethodGet:
		respond(nil)
	case http.MethodPost:
		var body UniSettings
		if err := decodeJSON(w, r, &body); err != nil {
			writeJSON(w, http.StatusBadRequest, map[string]any{"ok": false, "error": "bad json"})
			return
		}
		ctx, cancel := context.WithTimeout(r.Context(), 15*time.Second)
		defer cancel()
		if err := s.local.SetUniSettings(ctx, &body); err != nil {
			writeJSON(w, http.StatusOK, map[string]any{"ok": false, "error": err.Error()})
			return
		}
		select {
		case <-ctx.Done():
		case <-time.After(700 * time.Millisecond):
		}
		respond(map[string]any{"wrote": true})
	default:
		writeJSON(w, http.StatusMethodNotAllowed, map[string]string{"error": "GET or POST"})
	}
}
