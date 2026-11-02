package app

// Quiet hours - the window in which the robot will not start a scheduled clean
// and will not return to the dock to wash, because both are loud.
//
// The app calls this Do Not Disturb and sends it as data point 157. That path
// needs a broker, so this one does not use it. Writes go to
// eros.proto.msg.app.UndisturbedSetting, which is one of the channels the stack
// creates on demand: it does not exist until something publishes it, so a
// `shmget: no such file or directory` before the first write is the expected
// state and not a missing feature.
//
//	UndisturbedSetting { Header header = 1; UndisturbedRequest req = 2 }
//	UndisturbedRequest { Undisturbed undisturbed = 1 }
//	Undisturbed        { Switch sw = 1; TimePoint begin = 2; TimePoint end = 3 }
//	TimePoint          { uint32 hour = 1; uint32 minute = 2 }
//
// The request wrapper is not decoration and leaving it out is not a no-op. The
// data point records this setting as
//
//	2{1{1:1} 2{1:22} 3{1:8 2:30}}   = on, 22:00-08:30
//
// which reads as though field 2 were the Undisturbed message itself, and that
// is how this was first written. Publishing it that way on hardware set the
// switch to false and all four times to zero: the robot took the Switch
// submessage in field 1 as the whole Undisturbed message, got a varint where it
// wanted a message, and kept its defaults. A write that silently means "off,
// 00:00-00:00" is exactly the failure mds_pub cannot report, because it
// transmitted fine.
//
// So the frame below has one more level than the data point appears to, and the
// test pins it.
//
// Reads come from user_config.yaml rather than the channel, for the reason
// settings.go gives: the channel holds the last write and says nothing at all
// about a setting nobody has touched this boot.

import (
	"context"
	"fmt"
	"github.com/dhumpf/x10-ctl/internal/pb"
	"net/http"
	"strings"
	"time"
)

const mdsUndisturbed = "eros.proto.msg.app.UndisturbedSetting"

// TimePoint is a wall-clock minute. The robot's own clock decides when that is;
// see clock.go for where that clock comes from with no cloud.
type TimePoint struct {
	Hour   int `json:"hour"`
	Minute int `json:"minute"`
}

func (t TimePoint) valid() bool {
	return t.Hour >= 0 && t.Hour < 24 && t.Minute >= 0 && t.Minute < 60
}

func (t TimePoint) String() string { return fmt.Sprintf("%02d:%02d", t.Hour, t.Minute) }

// Undisturbed is the whole setting. Not a pointer-per-field like CleanSettings:
// a window is three values that only mean anything together, and a partial
// write of one end of it is a bug rather than a feature.
type Undisturbed struct {
	Enabled bool      `json:"enabled"`
	Begin   TimePoint `json:"begin"`
	End     TimePoint `json:"end"`
}

// Active reports whether now falls inside the window.
//
// The window wraps midnight - 22:00 to 08:30 is the default and the common case
// - so this is not a simple range test. Equal endpoints mean a zero-length
// window, not a whole day: the app cannot express "always" and neither can this.
func (u Undisturbed) Active(now time.Time) bool {
	if !u.Enabled {
		return false
	}
	cur := now.Hour()*60 + now.Minute()
	b := u.Begin.Hour*60 + u.Begin.Minute
	e := u.End.Hour*60 + u.End.Minute
	if b == e {
		return false
	}
	if b < e {
		return cur >= b && cur < e
	}
	return cur >= b || cur < e
}

// readUndisturbed reports the persisted quiet-hours window.
func readUndisturbed() (*Undisturbed, error) { return readUndisturbedFrom(userConfig) }

func readUndisturbedFrom(path string) (*Undisturbed, error) {
	sw, ok := scanKey(path, "undisturbed_switch")
	if !ok {
		return nil, fmt.Errorf("no undisturbed_switch in %s", path)
	}
	u := &Undisturbed{Enabled: yamlBool(sw)}
	get := func(key string) int {
		v, err := scanInt(path, key)
		if err != nil {
			return 0
		}
		return v
	}
	u.Begin = TimePoint{Hour: get("undisturbed_start_hour"), Minute: get("undisturbed_start_minute")}
	u.End = TimePoint{Hour: get("undisturbed_end_hour"), Minute: get("undisturbed_end_minute")}
	return u, nil
}

// encodeUndisturbed builds the UndisturbedSetting payload.
//
// Zero values are omitted, which is proto3 and not an optimisation: a minute of
// 0 is encoded as an absent field, and the robot's own record of the 22:00
// window shows begin as 2{1:22} with no field 2 at all.
func encodeUndisturbed(u Undisturbed) []byte {
	point := func(num uint64, t TimePoint) []byte {
		var p []byte
		if t.Hour != 0 {
			p = append(p, pb.FieldVarint(1, uint64(t.Hour))...)
		}
		if t.Minute != 0 {
			p = append(p, pb.FieldVarint(2, uint64(t.Minute))...)
		}
		return pb.FieldBytes(num, p)
	}
	var body []byte
	var sw []byte
	if u.Enabled {
		sw = pb.FieldVarint(1, 1)
	}
	body = append(body, pb.FieldBytes(1, sw)...)
	body = append(body, point(2, u.Begin)...)
	body = append(body, point(3, u.End)...)
	return pb.FieldBytes(2, pb.FieldBytes(1, body))
}

// SetUndisturbed publishes a quiet-hours window onto the bus.
func (l *Local) SetUndisturbed(ctx context.Context, u Undisturbed) error {
	if !u.Begin.valid() || !u.End.valid() {
		return fmt.Errorf("hour must be 0-23 and minute 0-59")
	}
	return l.mdsPublish(ctx, mdsUndisturbed, encodeUndisturbed(u))
}

func (s *server) handleUndisturbed(w http.ResponseWriter, r *http.Request) {
	respond := func(extra map[string]any) {
		cur, err := readUndisturbed()
		if err != nil {
			writeJSON(w, http.StatusServiceUnavailable, map[string]any{"ok": false, "error": err.Error()})
			return
		}
		out := map[string]any{
			"ok":          true,
			"undisturbed": cur,
			"active_now":  cur.Active(time.Now()),
			"window":      cur.Begin.String() + " - " + cur.End.String(),
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
		var body Undisturbed
		if err := decodeJSON(w, r, &body); err != nil {
			writeJSON(w, http.StatusBadRequest, map[string]any{"ok": false, "error": "bad json"})
			return
		}
		ctx, cancel := context.WithTimeout(r.Context(), 15*time.Second)
		defer cancel()
		if err := s.local.SetUndisturbed(ctx, body); err != nil {
			writeJSON(w, http.StatusOK, map[string]any{"ok": false, "error": err.Error()})
			return
		}
		// cloud_cli persists the change to user_config.yaml as part of handling
		// it, so an immediate read back returns the value from before.
		select {
		case <-ctx.Done():
		case <-time.After(700 * time.Millisecond):
		}
		respond(map[string]any{"wrote": true})
	default:
		writeJSON(w, http.StatusMethodNotAllowed, map[string]string{"error": "GET or POST"})
	}
}

// trimYAML strips the whitespace and quoting the config writer sometimes emits.
func trimYAML(v string) string { return strings.Trim(strings.TrimSpace(v), `"`) }

// yamlBool reads the robot's flavour of boolean. The config writer emits bare
// true and false, but quoted values turn up in files that have been through the
// app, so both are accepted.
func yamlBool(v string) bool {
	switch trimYAML(v) {
	case "true", "1", "yes":
		return true
	}
	return false
}
