package dp

// Data points - the vocabulary of the MQTT channel, and the reason it exists
// alongside 9668.
//
// The local socket carries a SocketTransData with no DP-number field, so E_DP
// can only ever mean one fixed payload: a ModeCtrlRequest. Beep, battery and
// drying are separate data points, and nothing on 9668 can address them. That
// is the whole justification for cloud.go.
//
// Encoding rule, from get_product_data_point: a DP declared Raw is
// base64(varint length + protobuf body); Bool, Value, Enum and String are plain
// JSON scalars. Sending base64 to a Bool DP is silently ignored - every attempt
// to beep with a protobuf body was dropped before {"160": true} worked.

import (
	"encoding/base64"
	"fmt"
	"github.com/dhumpf/x10-ctl/internal/pb"
	"sort"
	"strconv"
)

// Data point ids used here. The full table is in dp_decode.py; these are the
// ones this program reads or writes.
const (
	DPModeCtrl    = 152 // Raw   ModeCtrlRequest - same message 9668 carries
	DPWorkStatus  = 153 // Raw   state machine, field 2 is the state
	dpSuction     = 158 // Enum
	DPCallRobot   = 160 // Bool  beep / find robot
	DPVolume      = 161 // Value 0-100
	DPBatLevel    = 163 // Value 0-100, read only
	DPRoomParams  = 170 // Raw   per-room clean parameters, sent before a clean
	DPStation     = 173 // Raw   StationRequest: dock config and manual actions
	dpErrWarning  = 177 // Raw   fault codes
	dpCleanParams = 154 // Raw
)

var dpNames = map[int]string{
	150: "proto", 151: "power", 152: "mode_ctrl", 153: "work_status",
	154: "clean_params", 155: "remote_ctrl", 156: "pause_job", 157: "dnd",
	158: "suction_level", 159: "boost_iq", 160: "calling_robot", 161: "volume",
	162: "user_language", 163: "bat_level", 164: "timing", 165: "rooms",
	166: "log_debug", 167: "clean_statistics", 168: "consumables",
	169: "app_dev_info", 170: "map_edit", 171: "multi_maps_ctrl",
	172: "multi_maps_mng", 173: "station", 174: "media_manager",
	176: "unisetting", 177: "error_warning", 178: "toast", 179: "analysis",
	180: "scenes",
}

func DPName(id int) string {
	if n, ok := dpNames[id]; ok {
		return n
	}
	return "dp" + strconv.Itoa(id)
}

// StationRequest.manual_cmd (field 2) is a ManualActionCmd oneof, station.proto.
// `dust` was captured byte-identical to the app's own command and `dry` drove
// the station to DRYING; the rest are read off the same oneof, so the field
// numbers are the vendor's but the effect is untested per-unit.
var StationActions = map[string]uint64{
	"self-maintain": 1, "dry": 2, "dust": 3,
	"wash": 4, "descale": 5, "cut-hair": 6,
}

// DryCfg.Cfg and Duration.Level, both 0-2. There is no value 3 - the app shows
// 1/2/3 hours, which are the three Duration levels, not the enum's values.
var dryModes = map[string]uint64{"close": 0, "standard": 1, "quick": 2}
var dryDurations = map[string]uint64{"short": 0, "medium": 1, "long": 2}

// dpValue wraps a protobuf body the way a Raw DP expects.
func DPValue(body []byte) string {
	return base64.StdEncoding.EncodeToString(pb.Delim(body))
}

// stationAction builds a manual dock command - dry, dust, wash and friends.
//
//	StationRequest{ manual_cmd{ <action> = true } }
//
// No sequence number anywhere: the captured dust command carried none.
func StationAction(name string) (string, error) {
	num, ok := StationActions[name]
	if !ok {
		return "", fmt.Errorf("unknown station action %q", name)
	}
	return DPValue(pb.FieldBytes(2, pb.FieldVarint(num, 1))), nil
}

// dryConfig builds the persistent drying setting.
//
//	StationRequest{ auto_cfg{ dry = DryCfg{cfg, Duration{level}} } }
//
// The empty Duration for level 0 is deliberate: the captured STANDARD+SHORT
// command is 0a06120408011200, which carries field 2 present but empty rather
// than an explicit zero. Reproducing the vendor's bytes exactly costs nothing
// and removes one variable if the robot ever ignores a command.
func DryConfig(mode, duration string) (string, error) {
	cfg, ok := dryModes[mode]
	if !ok {
		return "", fmt.Errorf("unknown dry mode %q (close, standard, quick)", mode)
	}
	level, ok := dryDurations[duration]
	if !ok {
		return "", fmt.Errorf("unknown dry duration %q (short, medium, long)", duration)
	}
	dry := pb.FieldVarint(1, cfg)
	if level == 0 {
		dry = append(dry, pb.FieldBytes(2, nil)...)
	} else {
		dry = append(dry, pb.FieldBytes(2, pb.FieldVarint(1, level))...)
	}
	return DPValue(pb.FieldBytes(1, pb.FieldBytes(2, dry))), nil
}

// modeCtrlDP encodes the same ModeCtrlRequest client.go sends over 9668, so a
// movement command can go either way once MQTT is up.
func modeCtrlDP(method, seq uint64) string {
	return DPValue(append(pb.FieldVarint(1, method), pb.FieldVarint(2, seq)...))
}

// workStates are the values seen in work_status field 2. All four were observed;
// anything else is reported as its number rather than guessed at.
var WorkStates = map[uint64]string{
	1: "transient", 3: "docked", 5: "cleaning", 6: "remote_ctrl", 7: "returning",
}

// rawDPBody unwraps a Raw DP value: base64, then the varint length prefix.
func rawDPBody(v any) ([]byte, bool) {
	s, isString := v.(string)
	if !isString {
		return nil, false
	}
	raw, err := base64.StdEncoding.DecodeString(s)
	if err != nil || len(raw) == 0 {
		return nil, false
	}
	n, off, ok := pb.ReadVarint(raw, 0)
	if !ok {
		return nil, false
	}
	if off+int(n) <= len(raw) {
		return raw[off : off+int(n)], true
	}
	return raw[off:], true
}

// workStateOf pulls the state out of a work_status report - if this particular
// report carries one.
//
// Reports are partial. A robot off its dock sends work_status with fields 14
// and 15 and no field 2 at all; that is an update to something else, not a
// transition to an unknown state. Returning false lets the caller keep the last
// state it knew rather than overwriting "cleaning" with a field dump.
func WorkStateOf(v any) (string, bool) {
	body, ok := rawDPBody(v)
	if !ok {
		return "", false
	}
	for _, f := range pb.ParseFields(body) {
		if f.Num == 2 && f.Wire == 0 {
			if name, ok := WorkStates[f.Val]; ok {
				return name, true
			}
			return "state " + strconv.FormatUint(f.Val, 10), true
		}
	}
	return "", false
}

// decodeDP turns one reported value into something the status card can show.
// Raw DPs are structural - field numbers and values - because most of the
// .proto files are not public and a wrong label is worse than a number.
func DecodeDP(id int, v any) string {
	if id == DPWorkStatus {
		if state, ok := WorkStateOf(v); ok {
			return state
		}
	}
	body, ok := rawDPBody(v)
	if !ok {
		return fmt.Sprint(v)
	}
	return pbSummary(body)
}

// pbSummary renders a flat protobuf as "1:5 2:{...}" - enough to read state off
// the wire without a schema. Ported from dp_decode.pretty, one level deep only;
// the full recursive dump belongs in the Python tooling, not the status card.
func pbSummary(body []byte) string {
	out := ""
	for _, f := range pb.ParseFields(body) {
		if out != "" {
			out += " "
		}
		switch f.Wire {
		case 0:
			out += fmt.Sprintf("%d:%d", f.Num, f.Val)
		case 2:
			if Printable(f.Data) {
				out += fmt.Sprintf("%d:%q", f.Num, string(f.Data))
			} else {
				out += fmt.Sprintf("%d:%dB", f.Num, len(f.Data))
			}
		default:
			out += fmt.Sprintf("%d:%x", f.Num, f.Data)
		}
	}
	if out == "" {
		return "(empty)"
	}
	return out
}

func Printable(b []byte) bool {
	if len(b) == 0 {
		return false
	}
	for _, c := range b {
		if c < 32 || c > 126 {
			return false
		}
	}
	return true
}

// sortedDPs gives the ids of a report in a stable order, so the UI does not
// reshuffle on every refresh.
func SortedDPs(m map[int]string) []int {
	ids := make([]int, 0, len(m))
	for id := range m {
		ids = append(ids, id)
	}
	sort.Ints(ids)
	return ids
}

// Methods that need naming in code rather than a map lookup.
const (
	MethodAutoClean  uint64 = 0
	MethodRoomClean  uint64 = 1
	MethodRCClean    uint64 = 5
	MethodFastMap    uint64 = 9
	MethodZonesClean uint64 = 2
	MethodSpotClean  uint64 = 3

	// Ending a remote-drive session takes both: STOP_RC_CLEAN leaves the mode
	// and STOP_TASK is what actually retracts the mops. See brakeAndPark.
	MethodStopRCClean uint64 = 16
	MethodStop        uint64 = 12
)

// ParamField is the ModeCtrlRequest oneof field a method's param belongs in.
// Absent means the method takes none, and 4 is a safe default only because it
// is never reached without a param.
//
//	AutoClean 3, SelectRoomsClean 4, SelectZonesClean 5, SpotClean 6, Goto 7
func ParamField(method uint64) uint64 {
	switch method {
	case MethodAutoClean:
		return 3
	case MethodZonesClean:
		return 5
	case MethodSpotClean:
		return 6
	default:
		return 4
	}
}
