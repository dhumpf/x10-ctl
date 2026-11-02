package app

// A read-only window on the mds registry, for the debug page.
//
// Every read here is mdsNewest, which is a SHM_RDONLY attach, so picking the
// wrong channel out of the dropdown cannot disturb the running stack. Nothing
// on this path publishes.
//
// The point of it is that mds records carry no field names - only numbers - so
// learning a channel means looking at one. That used to mean building
// cmd/mdsdump, copying it to /tmp and shelling in. This puts the same walk
// behind an endpoint, which is also how the next channel gets reverse
// engineered: watch a value change and the field number is the one that moved.

import (
	"encoding/binary"
	"encoding/hex"
	"github.com/dhumpf/x10-ctl/internal/pb"
	"math"
	"net/http"
	"sort"
	"strings"
)

// mdsDebugChannels is what the dropdown offers: the channels this code already
// speaks, plus the segments named in akos-bus.md. A name that does not resolve
// is not an error - it is reported as not live and the entry stays, because
// "this channel exists but is not published right now" is itself an answer.
var mdsDebugChannels = []string{
	"eros.proto.msg.app.CleanSetting",
	"eros.proto.msg.app.CloudEvent",
	"eros.proto.msg.app.CloudNetwork",
	"eros.proto.msg.app.ControlNotify",
	"eros.proto.msg.app.FunctionalCtrl",
	"eros.proto.msg.app.MapEditRequest",
	"eros.proto.msg.app.MapEditResponse",
	"eros.proto.msg.app.MapUpdateNotify",
	"eros.proto.msg.app.ModeCtrlRequest",
	"eros.proto.msg.app.ModeCtrlResponse",
	"eros.proto.msg.app.MultiMapUpdateNotify",
	"eros.proto.msg.app.PathUpdateNotify",
	"eros.proto.msg.app.PoseUpdateNotify",
	"eros.proto.msg.app.StationSetting",
	"eros.proto.msg.app.StatusUpdateNotify",
	"eros.proto.msg.app.TimerRequest",
	"eros.proto.msg.app.TimerResponse",
	"eros.proto.msg.app.UndisturbedSetting",
	"eros.proto.msg.app.Unisetting",
	"eros.proto.msg.app.VoiceSetting",
	"eros.proto.msg.audio.Control",
	"eros.proto.msg.audio.LanguageSet",
	"eros.proto.msg.camera_svc.JpegData",
	"eros.proto.msg.chassis.BatteryException",
	"eros.proto.msg.chassis.BatteryInfo",
	"eros.proto.msg.chassis.ComponentCtrl",
	"eros.proto.msg.chassis.ComponentData",
	"eros.proto.msg.chassis.Event",
	"eros.proto.msg.chassis.HomeSignalV2",
	"eros.proto.msg.chassis.LedCtrl",
	"eros.proto.msg.chassis.MotionCtrl",
	"eros.proto.msg.chassis.MotionProtect",
	"eros.proto.msg.chassis.OdometerData",
	"eros.proto.msg.chassis.PowerCtrl",
	"eros.proto.msg.chassis.SensorData",
	"eros.proto.msg.chassis.WheelData",
	"eros.proto.msg.map.OccupancyGrid",
	"eros.proto.msg.mds.Control",
	"eros.proto.msg.power.Request",
	"eros.proto.msg.power.Response",
	"eros.proto.msg.powerd.CpuStatus",
	"eros.proto.msg.sensor.ImuSixData",
	"eros.proto.msg.sensor.LaserControl",
	"eros.proto.msg.sensor.LaserPowerProxy",
	"eros.proto.msg.sensor.LaserScan",
	"eros.proto.msg.sensor.LaserStatus",
	"eros.proto.msg.sensor.camera.Control",
	"eros.proto.msg.sensor.camera.Notify",
	"eros.proto.msg.sensor.tof.Control",
	"eros.proto.msg.station.Control",
	"eros.proto.msg.station.ControlResponse",
	"eros.proto.msg.station.DeviceStatus",
}

// mdsField is one decoded protobuf field. Type is what the bytes look like,
// not what they are declared as - there is no descriptor on this wire.
type mdsField struct {
	Field  int        `json:"field"`
	Type   string     `json:"type"`
	Value  any        `json:"value,omitempty"`
	Fields []mdsField `json:"fields,omitempty"`
}

// mdsWalk decodes a record far enough to look at. Same heuristics as
// cmd/mdsdump: a length-delimited field that is all printable is shown as a
// string, a long multiple of four as packed floats, anything else is descended
// into as a nested message.
func mdsWalk(b []byte, depth int) []mdsField {
	var out []mdsField
	if depth > 6 {
		return out
	}
	i := 0
	for i < len(b) {
		tag, ni, ok := pb.ReadVarint(b, i)
		if !ok {
			return out
		}
		i = ni
		f := mdsField{Field: int(tag >> 3)}
		switch tag & 7 {
		case 0:
			v, ni, ok := pb.ReadVarint(b, i)
			if !ok {
				return out
			}
			i = ni
			f.Type, f.Value = "varint", v
		case 5:
			if i+4 > len(b) {
				return out
			}
			u := binary.LittleEndian.Uint32(b[i : i+4])
			f.Type, f.Value = "fixed32", math.Float32frombits(u)
			i += 4
		case 1:
			if i+8 > len(b) {
				return out
			}
			u := binary.LittleEndian.Uint64(b[i : i+8])
			f.Type, f.Value = "fixed64", math.Float64frombits(u)
			i += 8
		case 2:
			n, ni, ok := pb.ReadVarint(b, i)
			if !ok || ni+int(n) > len(b) {
				return out
			}
			i = ni
			v := b[i : i+int(n)]
			i += int(n)
			switch {
			case n >= 64 && n%4 == 0:
				cnt := int(n) / 4
				vals := make([]float32, 0, 8)
				for k := 0; k < cnt && k < 8; k++ {
					vals = append(vals, math.Float32frombits(binary.LittleEndian.Uint32(v[k*4:])))
				}
				f.Type, f.Value = "packed float32", map[string]any{"count": cnt, "first": vals}
			case mdsPrintable(v):
				f.Type, f.Value = "string", string(v)
			case n == 0:
				f.Type, f.Value = "empty", ""
			default:
				f.Type = "message"
				f.Fields = mdsWalk(v, depth+1)
				if len(f.Fields) == 0 {
					f.Type, f.Value = "bytes", hex.EncodeToString(v)
				}
			}
		default:
			return out
		}
		out = append(out, f)
	}
	return out
}

func mdsPrintable(b []byte) bool {
	if len(b) == 0 {
		return false
	}
	for _, c := range b {
		if c < 0x20 || c > 0x7e {
			return false
		}
	}
	return true
}

// handleMDS lists the registry, or reads one channel from it.
//
// Names are restricted to the eros.proto.msg. prefix. Not for safety - the read
// is read-only whatever the name - but because anything else is a typo, and the
// key is a hash, so a typo would otherwise attach some unrelated segment and
// print convincing nonsense.
func (s *server) handleMDS(w http.ResponseWriter, r *http.Request) {
	name := r.URL.Query().Get("c")
	if name == "" {
		type entry struct {
			Name string `json:"name"`
			Live bool   `json:"live"`
		}
		names := append([]string(nil), mdsDebugChannels...)
		sort.Strings(names)
		out := make([]entry, 0, len(names))
		for _, n := range names {
			out = append(out, entry{Name: n, Live: mdsAvailable(n)})
		}
		writeJSON(w, http.StatusOK, map[string]any{"ok": true, "channels": out})
		return
	}
	if !strings.HasPrefix(name, "eros.proto.msg.") {
		writeJSON(w, http.StatusBadRequest, map[string]any{
			"ok": false, "error": "channel must start with eros.proto.msg."})
		return
	}
	rec, err := mdsNewest(name)
	if err != nil {
		writeJSON(w, http.StatusOK, map[string]any{"ok": false, "channel": name, "error": err.Error()})
		return
	}
	writeJSON(w, http.StatusOK, map[string]any{
		"ok": true, "channel": name, "bytes": len(rec), "fields": mdsWalk(rec, 0),
	})
}
