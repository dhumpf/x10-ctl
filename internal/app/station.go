package app

// The dock, on the bus.
//
// Drying and dust collection were the last things still needing the broker, for
// no better reason than that nobody had traced them. They land on
// eros.proto.msg.station.Control, which has been live all along - shmid 45,
// three attachers, published by node `peripherals`.
//
// The channel does not carry StationRequest. cloud_cli translates the DP into a
// numeric action, and the shape was read off the wire by sending each command
// over MQTT and watching what appeared:
//
//	station.Control { Header header = 1;
//	                  uint32 action = 2;    1 dry, 8 collect dust
//	                  uint32 start  = 3;    1 to run
//	                  uint32 gear   = 5;    dry only, observed 1
//	                  uint32 secs   = 6 }   dry 7200 = 2 h, dust 30
//
// Field 6 being a duration is also the stop: re-issuing dry with one second
// ends a running cycle, confirmed by `Station work bit: 0` and run_status
// dropping from 4 to 0. ModeCtrl STOP_TASK does not touch the dock at all.
//
// Dust was action 7 here until 2026-07-31 and that was simply wrong. Reading
// the app's own records off the ring with cmd/mdsdump and matching them to the
// bus by seq settled it: action 8 start 1 secs 30 produces `DUST_COLLECT!
// State = 1` and `DustCollect]data = 0x1e`, and action 8 alone stops it.
// Action 7 only ever logs `clean processing, type():7 value():1` and does
// nothing, which is why every empty from here reported ok and moved no dust.
//
// The Header is not needed. The app fills it in -
// Header { Src src = 1 { uint32 = 1, uint64 = 2 }, string node = 2, uint64 seq = 3 }
// with node "peripherals" - but ours goes out with field 1 absent and works.
//
// Only dry and dust are mapped. Wash, descale and cut-hair would each need the
// same probe, and probing them means running water through the dock, so they
// stay on the data-point path until there is a reason to spend that.

import (
	"context"
	"encoding/json"
	"fmt"
	"github.com/dhumpf/x10-ctl/internal/dp"
	"github.com/dhumpf/x10-ctl/internal/pb"
	"log"
	"net/http"
	"strings"
	"time"
)

const mdsStationControl = "eros.proto.msg.station.Control"

const (
	stationActionDry  = 1
	stationActionDust = 8

	// What the app asks for, and what the robot came back with when it did.
	defaultDrySeconds = 7200

	// The app's own empty sends 30, logged as DustCollect data = 0x1e.
	defaultDustSeconds = 30
)

// busStationActions are the ones with a confirmed encoding. Anything absent
// falls through to the data point rather than guessing at a dock that can also
// pump water.
// Dust is deliberately absent. station.Control action 8 empties the bin and
// stops there; the data point runs the hair cut and the voice prompt too.
var busStationActions = map[string]uint64{
	"dry": stationActionDry,
}

// StationAction runs a dock action over the bus.
//
// Both dry and dust need field 6. Zero means the robot's own default.
func (l *Local) StationAction(ctx context.Context, action uint64, secs int) error {
	body := append(pb.FieldVarint(2, action), pb.FieldVarint(3, 1)...)
	switch action {
	case stationActionDry:
		body = append(body, pb.FieldVarint(5, 1)...)
		if secs <= 0 {
			secs = defaultDrySeconds
		}
		body = append(body, pb.FieldVarint(6, uint64(secs))...)
	case stationActionDust:
		// Matches what the app sends. Not sufficient on its own: see the
		// header note in the file comment.
		if secs <= 0 {
			secs = defaultDustSeconds
		}
		body = append(body, pb.FieldVarint(6, uint64(secs))...)
	}
	return l.mdsPublish(ctx, mdsStationControl, body)
}

// StationStop ends a running dock action.
//
// Field 3 is start, and 0 is stop. That is not a guess: `control` logs two
// branches, "clean processing, type():N value():1" and "stop clean processing,
// type():N value():0", and the second was seen on the wire when something else
// stopped a collection.
//
// Proto3 omits a zero, so stopping is the action field alone. The previous
// StopDry sent a one-second dry instead - still a START - which is why the dry
// never stopped and `peripherals` simply re-issued it with the remaining
// duration.
func (l *Local) StationStop(ctx context.Context, action uint64) error {
	return l.mdsPublish(ctx, mdsStationControl, pb.FieldVarint(2, action))
}

// StopDry ends a running mop dry.
func (l *Local) StopDry(ctx context.Context) error {
	return l.StationStop(ctx, stationActionDry)
}

// StationState is node_sensor_msgs/StationStatus, the fields worth acting on.
// Field names are the bus's own - akostopic renders station_status as JSON, so
// this unmarshals directly rather than counting protobuf fields.
type StationState struct {
	State         int  `json:"state"`
	ErrorCode     int  `json:"error_code"`
	ErrorCode1    int  `json:"error_code1"`
	CleanLevel    int  `json:"clean_level"`
	DirtyLevel    int  `json:"dirty_level"`
	Turbidity     int  `json:"turbidity_value"`
	ChildLock     int  `json:"childlock"`
	Work          int  `json:"work"`
	CollectDust   int  `json:"features_collect_dust"`
	CutHair       int  `json:"features_cut_hair"`
	StationType   int  `json:"station_type"`
	WaterLevel    int  `json:"remaining_water_level"`
	WaterPercent  int  `json:"remaining_water_percent"`
	ConnectStatus int  `json:"connect_status"`
	Valid         bool `json:"valid"`
}

// StationStatus reads one sample off the chassis bus.
//
// Single shot rather than a watcher: nothing here needs it continuously, and
// akostopic has to be started and killed for each read.
func (l *Local) StationStatus(ctx context.Context) (StationState, error) {
	var st StationState
	if !l.Available() {
		return st, fmt.Errorf("not running on the robot")
	}
	// The bound belongs to the context, not to a `timeout` in the shell. See
	// Local.run: the shell version is what leaked a subscriber for 19 hours.
	ctx, cancel := context.WithTimeout(ctx, 8*time.Second)
	defer cancel()
	out, err := l.run(ctx, akosTopic+" echo /topic/chassis/station_status")
	if err != nil && len(out) == 0 {
		return st, fmt.Errorf("station status: %w", err)
	}
	// The station publishes two interleaved records and only one of them carries
	// the water fields; the other reports both as 0. Confirmed 2026-08-07 by
	// sequence number - even seq had level 0, odd seq had 3200, alternating
	// twice a second with every other field identical. Taking the first line
	// therefore reported an empty tank half the time, so the water fields are
	// merged by maximum across the whole capture and the rest come from the
	// last complete record.
	var best StationState
	for _, line := range strings.Split(string(out), "\n") {
		body, ok := parseAkosJSON(line)
		if !ok {
			continue
		}
		var one StationState
		if json.Unmarshal(body, &one) != nil {
			continue
		}
		if one.WaterLevel > best.WaterLevel {
			best.WaterLevel = one.WaterLevel
		}
		if one.WaterPercent > best.WaterPercent {
			best.WaterPercent = one.WaterPercent
		}
		level, percent := best.WaterLevel, best.WaterPercent
		best = one
		best.WaterLevel, best.WaterPercent = level, percent
		best.Valid = true
	}
	if !best.Valid {
		return st, fmt.Errorf("no station status on the bus")
	}
	return best, nil
}

// station runs a dock action, bus first. Shared by the button list and the
// /api/station route so they cannot drift - the buttons used to go straight to
// the broker and quietly skipped the bus path entirely.
//
// Dust is checked against the station before it is sent. The publish always
// succeeds and `control` always logs "clean processing", so without this the UI
// reported a cheerful ok for a dock that did nothing at all.
func (s *server) station(ctx context.Context, action string) (string, error) {
	// The publish always succeeds and control always logs "clean processing",
	// so without this the UI reported a cheerful ok for a dock that did nothing.
	//
	// A mop dry owns the dock and pre-empts this. Stopping the dry does not
	// help: `peripherals` re-issues DRY with the remaining duration inside the
	// same second, observed at 5867 s, and run_status never leaves 4.
	if action == "dust" && s.local.Available() && !s.noLocalCtl {
		if st, err := s.local.StationStatus(ctx); err == nil && st.Valid {
			if st.Work != 0 {
				return "", fmt.Errorf("the dock is busy (work %d) - a mop dry holds it for up "+
					"to two hours and restarts itself if stopped. Empty once it finishes, or "+
					"turn mop drying off first", st.Work)
			}
			// features_collect_dust and error_code 2048 are permanent on this
			// station and mean nothing: the app empties the bin with both set.
		}
	}
	return s.stationSend(ctx, action)
}

func (s *server) stationSend(ctx context.Context, action string) (string, error) {
	if code, ok := busStationActions[action]; ok && s.local.Available() && !s.noLocalCtl {
		if err := s.local.StationAction(ctx, code, 0); err == nil {
			return "local", nil
		}
		log.Printf("station %s: local failed, falling back", action)
	}
	if _, known := dp.StationActions[action]; !known {
		return "", fmt.Errorf("unknown action %q", action)
	}
	if !s.cloud.Ready() {
		return "", fmt.Errorf("%s has no bus encoding yet and mqtt is off", action)
	}
	return "mqtt", s.cloud.Station(action)
}

// error_code bits, mapped on hardware 2026-08-07 by pulling and filling each
// tank in turn. Bit 11 (2048) is permanent on this unit and still unexplained.
const (
	stationErrWaterEmpty = 1 << 1 // clean-water tank empty
	// Set when the wastewater tank is full OR not seated, cleared when it is
	// present and empty. One fault, two causes - it is not a "full" flag.
	stationErrWasteFault = 1 << 9
	stationErrNoDustBag  = 1 << 18 // dust bag not seated
)

func (s *server) handleStation(w http.ResponseWriter, r *http.Request) {
	// GET reports the dock. Everything here was decoded long before anything
	// served it, which is why the water level never reached a dashboard.
	if r.Method == http.MethodGet {
		ctx, cancel := context.WithTimeout(r.Context(), 15*time.Second)
		defer cancel()
		st, err := s.local.StationStatus(ctx)
		if err != nil {
			writeJSON(w, http.StatusServiceUnavailable, map[string]any{"ok": false, "error": err.Error()})
			return
		}
		writeJSON(w, http.StatusOK, map[string]any{
			"ok": true, "station": st,
			"water_empty": st.ErrorCode&stationErrWaterEmpty != 0,
			"waste_fault": st.ErrorCode&stationErrWasteFault != 0,
			"no_dust_bag": st.ErrorCode&stationErrNoDustBag != 0,
			"busy":        st.Work != 0,
		})
		return
	}
	if r.Method != http.MethodPost {
		writeJSON(w, http.StatusMethodNotAllowed, map[string]string{"error": "GET or POST"})
		return
	}
	action := strings.ToLower(strings.TrimSpace(r.URL.Query().Get("action")))
	secs := atoiDefault(r.URL.Query().Get("seconds"), 0)

	ctx, cancel := context.WithTimeout(r.Context(), 15*time.Second)
	defer cancel()

	if action == "stopdry" {
		if err := s.local.StopDry(ctx); err != nil {
			writeJSON(w, http.StatusOK, map[string]any{"ok": false, "error": err.Error()})
			return
		}
		writeJSON(w, http.StatusOK, map[string]any{"ok": true, "action": action, "bus": "local"})
		return
	}

	// A duration only means anything for dry, and only on the bus.
	if code, ok := busStationActions[action]; ok && secs > 0 && s.local.Available() && !s.noLocalCtl {
		if err := s.local.StationAction(ctx, code, secs); err != nil {
			writeJSON(w, http.StatusOK, map[string]any{"ok": false, "error": err.Error()})
			return
		}
		writeJSON(w, http.StatusOK, map[string]any{
			"ok": true, "action": action, "bus": "local", "seconds": secs,
		})
		return
	}
	via, err := s.station(ctx, action)
	if err != nil {
		writeJSON(w, http.StatusOK, map[string]any{"ok": false, "error": err.Error()})
		return
	}
	writeJSON(w, http.StatusOK, map[string]any{"ok": true, "action": action, "bus": via})
}
