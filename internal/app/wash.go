package app

// Automatic mop washing: how often the station backwashes the mop during a
// clean, and for how long.
//
// The frame here was read off the wire, not inferred. On 2026-08-06 the
// cloud_cli TLS capture was running while the setting was changed to "every
// 20 minutes" in the vendor app, and this is what the robot sent:
//
//	f1 { f1 { f1: 1, f3 { f1: 20 } }, f2 {}, f3: 1 }
//
// user_config then read back backwash_freq_mode: 1 and backwash_freq_interval:
// 20, which is what ties the field numbers to the names.
//
//	StationSetting { Header header = 1; <request> = 2 }
//	<request>      { AutoActionCfg = 1 }
//	AutoActionCfg  { WashCfg wash = 1; ...; CollectDustCfgV2 collectdust_v2 = 6 }
//	WashCfg        { BackwashFreq wash_freq = 1; Duration wash_duration = 2;
//	                 Cfg cfg = 3 }   // 0 close, 1 standard
//	BackwashFreq   { Mode mode = 1;  // 0 by partition, 1 by time, 2 by area
//	                 Duration duration = 2; Numerical time_or_area = 3 }
//	Numerical      { uint32 value = 1 }
//	Duration       { Level level = 1 }  // 0 short, 1 medium, 2 long
//
// wash_duration arrives as a PRESENT but EMPTY submessage. That is not noise.
// A scalar enum at 0 is proto3's default and gets omitted, so an empty body is
// the only way to say SHORT, the same reason dust.go's false Switch survives.

import (
	"context"
	"fmt"
	"github.com/dhumpf/x10-ctl/internal/pb"
	"net/http"
	"time"
)

// BackwashFreq.Mode wire values.
const (
	washModeByPartition = 0
	washModeByTime      = 1
	washModeByArea      = 2
)

// washFreqModes index the wire value of BackwashFreq.Mode.
var washFreqModes = []string{"by partition", "by time", "by area"}

// washDurationLevels index the wire value of Duration.Level.
var washDurationLevels = []string{"short", "medium", "long"}

func washModeName(mode int) string {
	if mode >= 0 && mode < len(washFreqModes) {
		return washFreqModes[mode]
	}
	return fmt.Sprintf("mode %d", mode)
}

func washDurationName(level int) string {
	if level >= 0 && level < len(washDurationLevels) {
		return washDurationLevels[level]
	}
	return fmt.Sprintf("level %d", level)
}

// WashSettings is the automatic mop-wash configuration.
type WashSettings struct {
	InMission bool `json:"in_mission"`
	Mode      int  `json:"mode"`
	Interval  int  `json:"interval"`
	Duration  int  `json:"duration"`
	Detergent bool `json:"detergent"`
}

func readWashSettings() (*WashSettings, error) {
	mode, err := scanInt(userConfig, "backwash_freq_mode")
	if err != nil {
		return nil, err
	}
	w := &WashSettings{Mode: mode}
	if v, err := scanInt(userConfig, "backwash_freq_interval"); err == nil {
		w.Interval = v
	}
	if v, err := scanInt(userConfig, "wash_duration"); err == nil {
		w.Duration = v
	}
	if v, ok := scanKey(userConfig, "enb_in_mission_backwash"); ok {
		w.InMission = yamlBool(v)
	}
	if v, ok := scanKey(userConfig, "detergent_sw"); ok {
		w.Detergent = yamlBool(v)
	}
	return w, nil
}

// encodeWash builds the StationSetting payload for the wash frequency.
//
// interval is minutes when mode is by time and square metres when mode is by
// area. It is omitted entirely when zero, which leaves the robot's value alone.
func encodeWash(mode, interval, duration int) []byte {
	var freq []byte
	if mode != 0 {
		freq = append(freq, pb.FieldVarint(1, uint64(mode))...)
	}
	if interval > 0 {
		freq = append(freq, pb.FieldBytes(3, pb.FieldVarint(1, uint64(interval)))...)
	}

	wash := pb.FieldBytes(1, freq)

	var dur []byte
	if duration != 0 {
		dur = pb.FieldVarint(1, uint64(duration))
	}
	wash = append(wash, pb.FieldBytes(2, dur)...)

	// cfg = STANDARD. There is no way to send CLOSE: it is 0, so proto3 drops
	// it and the write merges to "no change". See SetWash.
	wash = append(wash, pb.FieldVarint(3, 1)...)

	return pb.FieldBytes(2, pb.FieldBytes(1, pb.FieldBytes(1, wash)))
}

// SetWash publishes the automatic mop-wash frequency.
//
// Turning the feature OFF is not offered. WashCfg.cfg is a scalar enum whose
// CLOSE is 0, so proto3 omits it and the robot merges the write as "leave
// alone" - the same trap that made AutoActionCfg.dry unsettable. Off has to go
// through the vendor app until something confirms another lever.
func (l *Local) SetWash(ctx context.Context, mode, interval, duration int) error {
	if mode < 0 || mode >= len(washFreqModes) {
		return fmt.Errorf("mode must be 0..%d", len(washFreqModes)-1)
	}
	if duration < 0 || duration >= len(washDurationLevels) {
		return fmt.Errorf("duration must be 0..%d", len(washDurationLevels)-1)
	}
	if interval < 0 || interval > 1000 {
		return fmt.Errorf("interval out of range")
	}
	return l.mdsPublish(ctx, mdsStationSetting, encodeWash(mode, interval, duration))
}

func (s *server) handleWash(w http.ResponseWriter, r *http.Request) {
	respond := func(extra map[string]any) {
		cur, err := readWashSettings()
		if err != nil {
			writeJSON(w, http.StatusServiceUnavailable, map[string]any{"ok": false, "error": err.Error()})
			return
		}
		out := map[string]any{
			"ok": true, "wash": cur,
			"mode_name":     washModeName(cur.Mode),
			"duration_name": washDurationName(cur.Duration),
			"note":          "frequency is written; off is not settable from here, use the app",
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
			Mode     *int `json:"mode"`
			Interval *int `json:"interval"`
			Duration *int `json:"duration"`
		}
		if err := decodeJSON(w, r, &body); err != nil || (body.Mode == nil && body.Interval == nil) {
			writeJSON(w, http.StatusBadRequest, map[string]any{
				"ok": false,
				"error": "want {\"mode\": 0|1|2, \"interval\": N} " +
					"where interval is minutes for mode 1 and square metres for mode 2",
			})
			return
		}
		cur, err := readWashSettings()
		if err != nil {
			writeJSON(w, http.StatusServiceUnavailable, map[string]any{"ok": false, "error": err.Error()})
			return
		}
		mode, interval, duration := cur.Mode, cur.Interval, cur.Duration
		if body.Mode != nil {
			mode = *body.Mode
		}
		if body.Interval != nil {
			interval = *body.Interval
		}
		if body.Duration != nil {
			duration = *body.Duration
		}

		ctx, cancel := context.WithTimeout(r.Context(), 15*time.Second)
		defer cancel()
		if err := s.local.SetWash(ctx, mode, interval, duration); err != nil {
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
