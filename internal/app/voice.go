package app

// Voice volume, on the bus.
//
// It was reachable only as data point 161 until now, which meant it was
// reachable only with a broker running. That is a bad dependency for this
// particular setting: writing almost anything else makes the robot say
// something, so the volume is the one control you want first and it was the one
// that needed the most machinery.
//
//	VoiceSetting { Header header = 1; Volume volume = 2 }
//	Volume       { uint32 level = 1 }
//
// Field 2 is not a guess. The channel is live on this robot with a single
// record in it reading f2{f1:10}, and user_config.yaml reads voice_volume: 10.
//
// 0 is silent, and silent is worth having: it is what makes it safe to test a
// write path at two in the morning.

import (
	"context"
	"fmt"
	"github.com/dhumpf/x10-ctl/internal/pb"
	"net/http"
	"time"
)

const mdsVoiceSetting = "eros.proto.msg.app.VoiceSetting"

func readVolume() (int, error) { return scanInt(userConfig, "voice_volume") }

// SetVolume publishes a voice volume onto the bus.
func (l *Local) SetVolume(ctx context.Context, level int) error {
	if level < 0 || level > 100 {
		return fmt.Errorf("volume %d out of range 0-100", level)
	}
	var vol []byte
	if level != 0 {
		vol = pb.FieldVarint(1, uint64(level))
	}
	return l.mdsPublish(ctx, mdsVoiceSetting, pb.FieldBytes(2, vol))
}

func (s *server) handleVolume(w http.ResponseWriter, r *http.Request) {
	respond := func(extra map[string]any) {
		out := map[string]any{"ok": true}
		if v, err := readVolume(); err == nil {
			out["volume"] = v
		} else {
			out["volume"] = nil
			out["note"] = err.Error()
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
			Volume *int `json:"volume"`
		}
		if err := decodeJSON(w, r, &body); err != nil || body.Volume == nil {
			writeJSON(w, http.StatusBadRequest, map[string]any{"ok": false, "error": "want {\"volume\": 0-100}"})
			return
		}
		ctx, cancel := context.WithTimeout(r.Context(), 15*time.Second)
		defer cancel()
		if err := s.local.SetVolume(ctx, *body.Volume); err != nil {
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
