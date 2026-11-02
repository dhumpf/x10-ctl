package app

// Off-peak charging: the window in which the robot is allowed to charge.
//
// It rides in the same message as the switch block, `Unisetting`, at request
// field 22 - one past `high_detection` at 21, which is why it was never found
// by walking the switch list.
//
// Read off the wire on 2026-08-06 with the window set to 01:17 to 16:43 in the
// vendor app. Deliberately odd numbers: four fields that all read 0 or 6 with a
// tidy window are impossible to tell apart.
//
//	b2 01 10                       field 22, 16 bytes
//	  0a 02 08 01                  1: Switch  { value: 1 }
//	  12 04 08 01 10 11            2: TimePoint { hour: 1,  minute: 17 }
//	  1a 04 08 10 10 2b            3: TimePoint { hour: 16, minute: 43 }
//
// user_config then read back peak_charge_switch true, start 1:17, end 16:43,
// so all five map one to one. Hours are 24-hour: 4 pm arrived as 16, not 4.
//
// TimePoint is the same shape quiet hours uses, `{ uint32 hour = 1; uint32
// minute = 2 }`, which is the one piece of this that was not a surprise.

import (
	"context"
	"fmt"
	"github.com/dhumpf/x10-ctl/internal/pb"
	"net/http"
	"time"
)

// peakChargeField is request numbering, like the rest of unisetting.go. See the
// renumbering trap documented there before reading any response.
const peakChargeField = 22

// PeakCharge is the off-peak charging window.
type PeakCharge struct {
	Enabled     bool `json:"enabled"`
	StartHour   int  `json:"start_hour"`
	StartMinute int  `json:"start_minute"`
	EndHour     int  `json:"end_hour"`
	EndMinute   int  `json:"end_minute"`
}

func readPeakCharge() (*PeakCharge, error) {
	sw, ok := scanKey(userConfig, "peak_charge_switch")
	if !ok {
		return nil, fmt.Errorf("no peak_charge_switch in %s", userConfig)
	}
	p := &PeakCharge{Enabled: yamlBool(sw)}
	for key, dst := range map[string]*int{
		"peak_charge_start_hour":   &p.StartHour,
		"peak_charge_start_minute": &p.StartMinute,
		"peak_charge_end_hour":     &p.EndHour,
		"peak_charge_end_minute":   &p.EndMinute,
	} {
		if v, err := scanInt(userConfig, key); err == nil {
			*dst = v
		}
	}
	return p, nil
}

func timePoint(hour, minute int) []byte {
	var b []byte
	if hour != 0 {
		b = append(b, pb.FieldVarint(1, uint64(hour))...)
	}
	if minute != 0 {
		b = append(b, pb.FieldVarint(2, uint64(minute))...)
	}
	return b
}

// encodePeakCharge builds the Unisetting payload for the charging window.
//
// The Switch is a message, so an off switch is an empty submessage rather than
// an absent one - present is the signal, exactly as in the switch block. The
// hour and minute inside a TimePoint are scalars and do vanish at zero, which
// is correct: midnight is the default the robot already holds.
func encodePeakCharge(p *PeakCharge) []byte {
	var sw []byte
	if p.Enabled {
		sw = pb.FieldVarint(1, 1)
	}
	body := pb.FieldBytes(1, sw)
	body = append(body, pb.FieldBytes(2, timePoint(p.StartHour, p.StartMinute))...)
	body = append(body, pb.FieldBytes(3, timePoint(p.EndHour, p.EndMinute))...)

	return pb.FieldBytes(2, pb.FieldBytes(peakChargeField, body))
}

// SetPeakCharge publishes the off-peak charging window.
func (l *Local) SetPeakCharge(ctx context.Context, p *PeakCharge) error {
	for _, c := range []struct {
		name string
		v    int
		max  int
	}{
		{"start_hour", p.StartHour, 23}, {"start_minute", p.StartMinute, 59},
		{"end_hour", p.EndHour, 23}, {"end_minute", p.EndMinute, 59},
	} {
		if c.v < 0 || c.v > c.max {
			return fmt.Errorf("%s must be 0..%d", c.name, c.max)
		}
	}
	return l.mdsPublish(ctx, mdsUnisetting, encodePeakCharge(p))
}

func (s *server) handlePeakCharge(w http.ResponseWriter, r *http.Request) {
	respond := func(extra map[string]any) {
		cur, err := readPeakCharge()
		if err != nil {
			writeJSON(w, http.StatusServiceUnavailable, map[string]any{"ok": false, "error": err.Error()})
			return
		}
		out := map[string]any{
			"ok": true, "peak_charge": cur,
			"window": fmt.Sprintf("%02d:%02d-%02d:%02d", cur.StartHour,
				cur.StartMinute, cur.EndHour, cur.EndMinute),
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
		cur, err := readPeakCharge()
		if err != nil {
			writeJSON(w, http.StatusServiceUnavailable, map[string]any{"ok": false, "error": err.Error()})
			return
		}
		body := struct {
			Enabled     *bool `json:"enabled"`
			StartHour   *int  `json:"start_hour"`
			StartMinute *int  `json:"start_minute"`
			EndHour     *int  `json:"end_hour"`
			EndMinute   *int  `json:"end_minute"`
		}{}
		if err := decodeJSON(w, r, &body); err != nil {
			return
		}
		next := *cur
		if body.Enabled != nil {
			next.Enabled = *body.Enabled
		}
		for dst, src := range map[*int]*int{
			&next.StartHour: body.StartHour, &next.StartMinute: body.StartMinute,
			&next.EndHour: body.EndHour, &next.EndMinute: body.EndMinute,
		} {
			if src != nil {
				*dst = *src
			}
		}

		ctx, cancel := context.WithTimeout(r.Context(), 15*time.Second)
		defer cancel()
		if err := s.local.SetPeakCharge(ctx, &next); err != nil {
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
