package app

// Mop drying, as a setting rather than an action.
//
// dust.go records this as unsettable: `AutoActionCfg.dry` was published on
// hardware with CLOSE and with QUICK, neither changed `dry_setting_sw` or
// `dry_setting_duration`, and the conclusion drawn was that field 2 is not the
// dry setting or the receiver ignores it.
//
// That conclusion was wrong, and the reason is the same proto3 trap wash.go
// hit. CLOSE is 0, so it is dropped from the message and merges as "leave
// alone" - the write was always empty. QUICK would have worked, but nothing
// was sent alongside it.
//
// Read off the wire on 2026-08-06 while the setting was changed in the vendor
// app from the 2 h default to 3 h. The app sends cfg AND duration together:
//
//	f1 { f2 { f1: 1, f2 { f1: 1 } } }   = 12 06 08 01 12 02 08 01
//
//	AutoActionCfg { WashCfg wash = 1; DryCfg dry = 2; ... }
//	DryCfg        { Cfg cfg = 1;       // 0 close, 1 standard, 2 quick
//	                Duration duration = 2 }
//	Duration      { Level level = 1 }  // 0 short, 1 medium, 2 long
//
// Level 1 is 3 hours, confirmed from both ends: the app put level 1 on the wire
// and user_config then read dry_setting_duration: 1. The 2 h default sends no
// level at all, so 2 h is level 0. Level 2 is untested - it is presumably the
// longest option the app offers, but nobody has selected it.

import (
	"context"
	"fmt"
	"github.com/dhumpf/x10-ctl/internal/pb"
	"net/http"
	"time"
)

// DryCfg.Cfg wire values.
const (
	dryClose    = 0
	dryStandard = 1
	dryQuick    = 2
)

var dryCfgNames = []string{"off", "standard", "quick"}

// Duration.Level wire values, shared with the wash duration.
var dryLevelNames = []string{"short", "medium", "long"}

func dryCfgName(v int) string {
	if v >= 0 && v < len(dryCfgNames) {
		return dryCfgNames[v]
	}
	return fmt.Sprintf("cfg %d", v)
}

// DrySettings is the automatic mop-dry configuration.
type DrySettings struct {
	Enabled  bool `json:"enabled"`
	Duration int  `json:"duration"`
}

func readDrySettings() (*DrySettings, error) {
	sw, ok := scanKey(userConfig, "dry_setting_sw")
	if !ok {
		return nil, fmt.Errorf("no dry_setting_sw in %s", userConfig)
	}
	d := &DrySettings{Enabled: yamlBool(sw)}
	if v, err := scanInt(userConfig, "dry_setting_duration"); err == nil {
		d.Duration = v
	}
	return d, nil
}

// encodeDry builds the StationSetting payload for the dry configuration.
//
// cfg is always emitted, including STANDARD, because that is what makes the
// message non-empty. Sending a bare duration would merge as "leave alone" for
// the same reason CLOSE always did.
func encodeDry(cfg, level int) []byte {
	var dry []byte
	if cfg != 0 {
		dry = append(dry, pb.FieldVarint(1, uint64(cfg))...)
	}
	var dur []byte
	if level != 0 {
		dur = pb.FieldVarint(1, uint64(level))
	}
	dry = append(dry, pb.FieldBytes(2, dur)...)

	return pb.FieldBytes(2, pb.FieldBytes(1, pb.FieldBytes(2, dry)))
}

// SetDry publishes the automatic mop-dry configuration.
//
// Turning drying OFF is not offered here, for the reason in the header: CLOSE
// is 0 and proto3 drops it, so the write would merge as "no change" and report
// success while doing nothing. Use the app, or stop a running cycle with
// StopDry, which is a station action and does work.
func (l *Local) SetDry(ctx context.Context, cfg, level int) error {
	if cfg <= 0 || cfg >= len(dryCfgNames) {
		return fmt.Errorf("cfg must be 1 (standard) or 2 (quick); 0 cannot be sent")
	}
	if level < 0 || level >= len(dryLevelNames) {
		return fmt.Errorf("duration must be 0..%d", len(dryLevelNames)-1)
	}
	return l.mdsPublish(ctx, mdsStationSetting, encodeDry(cfg, level))
}

func (s *server) handleDry(w http.ResponseWriter, r *http.Request) {
	respond := func(extra map[string]any) {
		cur, err := readDrySettings()
		if err != nil {
			writeJSON(w, http.StatusServiceUnavailable, map[string]any{"ok": false, "error": err.Error()})
			return
		}
		out := map[string]any{
			"ok": true, "dry": cur,
			"duration_name": washDurationName(cur.Duration),
			"note":          "off is not settable from here, use the app; /api/station stopdry ends a running cycle",
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
			Cfg      *int `json:"cfg"`
			Duration *int `json:"duration"`
		}
		if err := decodeJSON(w, r, &body); err != nil || (body.Cfg == nil && body.Duration == nil) {
			writeJSON(w, http.StatusBadRequest, map[string]any{
				"ok":    false,
				"error": "want {\"cfg\": 1|2, \"duration\": 0|1|2}; cfg 0 (off) cannot be encoded",
			})
			return
		}
		cur, err := readDrySettings()
		if err != nil {
			writeJSON(w, http.StatusServiceUnavailable, map[string]any{"ok": false, "error": err.Error()})
			return
		}
		cfg, level := dryStandard, cur.Duration
		if body.Cfg != nil {
			cfg = *body.Cfg
		}
		if body.Duration != nil {
			level = *body.Duration
		}

		ctx, cancel := context.WithTimeout(r.Context(), 15*time.Second)
		defer cancel()
		if err := s.local.SetDry(ctx, cfg, level); err != nil {
			writeJSON(w, http.StatusOK, map[string]any{"ok": false, "error": err.Error()})
			return
		}
		select {
		case <-ctx.Done():
		case <-time.After(700 * time.Millisecond):
		}
		respond(map[string]any{"wrote": true, "cfg_name": dryCfgName(cfg)})
	default:
		writeJSON(w, http.StatusMethodNotAllowed, map[string]string{"error": "GET or POST"})
	}
}
