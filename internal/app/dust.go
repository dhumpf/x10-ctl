package app

// Automatic dust emptying: whether the station empties the robot's bin into the
// bag after a clean, and how often.
//
// The one-shot "Empty dustbin" button is a station ACTION (station.go, action
// 7). This is the setting behind it, and they are different things: the action
// runs the fan now, the setting decides whether it runs on its own.
//
//	StationSetting    { Header header = 1; <request> = 2 }
//	<request>         { <cfg> = 1 }
//	<cfg>             { ...; CollectDustCfgV2 collectdust_v2 = 6; ... }
//	CollectDustCfgV2  { Switch sw = 1; Mode mode = 2 }
//	Mode              { Value value = 1;  // 0 by task, 1 by time
//	                    uint32 task = 2; uint32 time = 3 }
//
// Field 6 is not inferred from the proto alone. The live channel carried
// exactly one record before the robot rebooted, and it was the write that
// turned this off:
//
//	f2 { f1 { f6 { f1 {} } } }   =  12 06 0a 04 32 02 0a 00
//
// An empty Switch is proto3 for false. encodeCollectDust reproduces that frame
// byte for byte, which is the only reason the ON frame is worth trusting.
//
// The channel does not exist until something publishes it, so a missing segment
// after a reboot is the normal state rather than a fault.

import (
	"context"
	"fmt"
	"github.com/dhumpf/x10-ctl/internal/pb"
	"net/http"
	"time"
)

const mdsStationSetting = "eros.proto.msg.app.StationSetting"

// collectDustModes index by the wire value of Mode.Value in CollectDustCfgV2.
var collectDustModes = []string{"by task", "by time"}

// The older CollectDustCfg { CLOSE = 0; ONCE = 1; TWICE = 2 }, kept only to name
// a 2 that predates a V2 write. collectdust_mode read 2 before 2026-08-10.
var collectDustLegacy = []string{"off", "after every clean", "after every 2 cleans"}

// collectdust_mode is the V2 enum. Writing Mode.value 1 moved it 0 -> 1 on
// hardware, so it is not the legacy CLOSE/ONCE/TWICE it was read as.
func dustModeName(mode int) string {
	if mode >= 0 && mode < len(collectDustModes) {
		return collectDustModes[mode]
	}
	if mode >= 0 && mode < len(collectDustLegacy) {
		return collectDustLegacy[mode]
	}
	return fmt.Sprintf("mode %d", mode)
}

// dustHow says when the station will empty, in words.
func dustHow(d *DustSettings) string {
	if !d.Enabled {
		return "off"
	}
	switch {
	case d.Mode == 1 && d.Minutes > 0:
		return fmt.Sprintf("every %d min", d.Minutes)
	case d.Mode == 1:
		return "by time, no interval set"
	case d.Task == 1:
		return "after every clean"
	case d.Task > 1:
		return fmt.Sprintf("after every %d cleans", d.Task)
	}
	return "by task, no count set"
}

// DustSettings is the automatic-empty configuration.
type DustSettings struct {
	Enabled bool `json:"enabled"`
	Mode    int  `json:"mode"`
	Task    int  `json:"task"`
	Minutes int  `json:"minutes"`
}

func readDustSettings() (*DustSettings, error) {
	sw, ok := scanKey(userConfig, "collectdust_sw")
	if !ok {
		return nil, fmt.Errorf("no collectdust_sw in %s", userConfig)
	}
	d := &DustSettings{Enabled: yamlBool(sw)}
	if v, err := scanInt(userConfig, "collectdust_mode"); err == nil {
		d.Mode = v
	}
	if v, err := scanInt(userConfig, "collectdust_task"); err == nil {
		d.Task = v
	}
	if v, err := scanInt(userConfig, "collectdust_time"); err == nil {
		d.Minutes = v
	}
	return d, nil
}

// encodeCollectDust builds the StationSetting payload for the switch alone.
//
// Only the switch goes out here, byte for byte against the recorded frame.
// Frequency lives in encodeCollectDustFull below.
func encodeCollectDust(on bool) []byte {
	var sw []byte
	if on {
		sw = pb.FieldVarint(1, 1)
	}
	cfg := pb.FieldBytes(1, sw)    // CollectDustCfgV2.sw
	inner := pb.FieldBytes(6, cfg) // <cfg>.collectdust_v2
	return pb.FieldBytes(2, pb.FieldBytes(1, inner))
}

// encodeCollectDustFull sends the switch AND the Mode submessage.
//
//	CollectDustCfgV2 { Switch sw = 1; Mode mode = 2 }
//	Mode { Value value = 1;  // 0 by task, 1 by time
//	       uint32 task = 2; uint32 time = 3 }
//
// Confirmed on hardware 2026-08-10: task 2 -> 1, time 2 -> 0 and value 0 -> 1
// each landed in user_config, so all three field numbers are right.
func encodeCollectDustFull(on bool, mode, task, minutes int) []byte {
	var sw []byte
	if on {
		sw = pb.FieldVarint(1, 1)
	}
	var m []byte
	if mode != 0 {
		m = append(m, pb.FieldVarint(1, uint64(mode))...)
	}
	if task > 0 {
		m = append(m, pb.FieldVarint(2, uint64(task))...)
	}
	if minutes > 0 {
		m = append(m, pb.FieldVarint(3, uint64(minutes))...)
	}
	cfg := append(pb.FieldBytes(1, sw), pb.FieldBytes(2, m)...)
	return pb.FieldBytes(2, pb.FieldBytes(1, pb.FieldBytes(6, cfg)))
}

// SetCollectDustFull publishes the switch together with its mode.
func (l *Local) SetCollectDustFull(ctx context.Context, on bool, mode, task, minutes int) error {
	return l.mdsPublish(ctx, mdsStationSetting, encodeCollectDustFull(on, mode, task, minutes))
}

// SetCollectDust publishes the automatic dust-empty switch.
func (l *Local) SetCollectDust(ctx context.Context, on bool) error {
	return l.mdsPublish(ctx, mdsStationSetting, encodeCollectDust(on))
}

// encodeCollectDustLegacy writes AutoActionCfg.collectdust, field 3.
//
//	CollectDustCfg { Cfg cfg = 1 }   // 0 close, 1 once, 2 twice
//
// The legacy config, distinct from collectdust_v2 at field 6. Published on
// hardware 2026-08-10 and ignored: collectdust_mode did not move. Use the V2
// path instead; this stays as the record that field 3 is a dead end.
func encodeCollectDustLegacy(mode int) []byte {
	var cfg []byte
	if mode != 0 {
		cfg = pb.FieldVarint(1, uint64(mode))
	}
	return pb.FieldBytes(2, pb.FieldBytes(1, pb.FieldBytes(3, cfg)))
}

// SetCollectDustLegacy publishes the legacy collect-dust config.
func (l *Local) SetCollectDustLegacy(ctx context.Context, mode int) error {
	if mode < 0 || mode > 2 {
		return fmt.Errorf("legacy mode must be 0..2")
	}
	return l.mdsPublish(ctx, mdsStationSetting, encodeCollectDustLegacy(mode))
}

// Mop drying is NOT settable from here, and this is the record of trying.
//
// The dock will not empty the bin while it is drying: `peripherals` re-issues
// StationControl DRY with the remaining duration inside the same second, so
// stopping the dry on station.Control never sticks and run_status stays 4.
//
// The obvious lever looked like AutoActionCfg.dry, field 2 of the same
// container whose field 6 drives the dust switch:
//
//	AutoActionCfg { WashCfg wash = 1; DryCfg dry = 2; ...
//	                CollectDustCfgV2 collectdust_v2 = 6 }
//	DryCfg        { Cfg cfg = 1;  // 0 close, 1 standard, 2 quick
//	                Duration duration = 2 }
//
// Both CLOSE and QUICK were published on hardware. Neither changed
// dry_setting_sw or dry_setting_duration, and neither shortened the running
// cycle. Field 6 does work, so the container is right and field 2 is not the
// dry setting - or the receiver ignores it.
//
// CLOSE could never have worked anyway: DryCfg.cfg is a scalar enum, so 0 is
// proto3's default and is omitted, leaving an empty submessage that merges to
// "no change". A false Switch survives only because Switch is a MESSAGE, and
// its presence is the signal.
//
// No endpoint ships for this. Turn mop drying off in the vendor app, or wait
// for the cycle to finish.

func (s *server) handleDust(w http.ResponseWriter, r *http.Request) {
	respond := func(extra map[string]any) {
		cur, err := readDustSettings()
		if err != nil {
			writeJSON(w, http.StatusServiceUnavailable, map[string]any{"ok": false, "error": err.Error()})
			return
		}
		out := map[string]any{
			"ok": true, "dust": cur,
			"mode_name": dustModeName(cur.Mode),
			"how":       dustHow(cur),
			"note":      "switch, mode, task and interval all write; the dock still will not empty while it dries mops",
			"dry_setting_sw": func() any {
				if v, ok := scanKey(userConfig, "dry_setting_sw"); ok {
					return yamlBool(v)
				}
				return nil
			}(),
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
		var body struct {
			Enabled *bool `json:"enabled"`
			Legacy  *int  `json:"legacy_mode"`
			Full    bool  `json:"full"`
			Mode    *int  `json:"mode"`
			Task    *int  `json:"task"`
			Minutes *int  `json:"minutes"`
		}
		freq := func() bool { return body.Mode != nil || body.Task != nil || body.Minutes != nil }
		if err := decodeJSON(w, r, &body); err != nil || (body.Enabled == nil && body.Legacy == nil && !freq()) {
			writeJSON(w, http.StatusBadRequest, map[string]any{
				"ok": false, "error": "want {\"enabled\": true|false}, {\"legacy_mode\": 0|1|2} " +
					"or {\"mode\": 0|1, \"task\": N, \"minutes\": N}",
			})
			return
		}
		if body.Mode != nil && (*body.Mode < 0 || *body.Mode > 1) {
			writeJSON(w, http.StatusBadRequest, map[string]any{
				"ok": false, "error": "mode is 0 by task or 1 by time",
			})
			return
		}
		if (body.Task != nil && (*body.Task < 0 || *body.Task > 100)) ||
			(body.Minutes != nil && (*body.Minutes < 0 || *body.Minutes > 1440)) {
			writeJSON(w, http.StatusBadRequest, map[string]any{
				"ok": false, "error": "task is 0..100, minutes is 0..1440",
			})
			return
		}
		ctx, cancel := context.WithTimeout(r.Context(), 15*time.Second)
		defer cancel()
		if body.Legacy != nil {
			if err := s.local.SetCollectDustLegacy(ctx, *body.Legacy); err != nil {
				writeJSON(w, http.StatusOK, map[string]any{"ok": false, "error": err.Error()})
				return
			}
		}
		if body.Enabled != nil || freq() {
			cur, _ := readDustSettings()
			on := cur != nil && cur.Enabled
			if body.Enabled != nil {
				on = *body.Enabled
			}
			var err error
			switch {
			case freq():
				// Chosen frequency, not the read-back echo. Mode's field numbers
				// are inferred from the proto, unlike the switch at field 6.
				mode, task, minutes := 0, 0, 0
				if cur != nil {
					mode, task, minutes = cur.Mode, cur.Task, cur.Minutes
				}
				if body.Mode != nil {
					mode = *body.Mode
				}
				if body.Task != nil {
					task = *body.Task
				}
				if body.Minutes != nil {
					minutes = *body.Minutes
				}
				err = s.local.SetCollectDustFull(ctx, on, mode, task, minutes)
			case body.Full && cur != nil:
				err = s.local.SetCollectDustFull(ctx, on, cur.Mode, cur.Task, cur.Minutes)
			default:
				err = s.local.SetCollectDust(ctx, on)
			}
			if err != nil {
				writeJSON(w, http.StatusOK, map[string]any{"ok": false, "error": err.Error()})
				return
			}
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
