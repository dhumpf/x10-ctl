package app

// What the robot can feel.
//
// eros.proto.msg.chassis.SensorData, published by chassis-t2320. The shape
// comes from the factory-test protos, which describe the same hardware:
//
//	SensorData { Ground ground = 1;          // cliff sensors, per-sensor bools
//	             Bump bump = 2;              // BR, BL - the physical bumper
//	             RadarBump radar_bump = 3;   // RBR, RBL, RBM
//	             Tof tof = 4;                // float dist, state, return rate
//	             Switch dust_box_state = 5;
//	             Switch mop_holder_state_l = 6;
//	             Switch mop_holder_state_r = 7;
//	             FrontWall front_wall = 8;
//	             Switch water_box_state = 9;
//	             Charge charge = 10;
//	             UltrasoundData ultrasound = 11 }
//	Bump      { repeated bool state = 1 }
//	Ground    { repeated bool state = 1; repeated sint32 data = 2 }
//	FrontWall { uint32 state = 1; repeated sint32 data = 2 }
//
// The bools are packed, so they arrive as one length-delimited field holding a
// run of varints rather than as separate fields. Handing that to the generic
// field walker produces nonsense - a field number of zero, which is not legal
// protobuf - so they are read directly.

import (
	"context"
	"github.com/dhumpf/x10-ctl/internal/pb"
	"math"
	"net/http"
	"time"
)

// Sensors is the subset worth showing while driving by hand.
type Sensors struct {
	BumpLeft   bool `json:"bump_left"`
	BumpRight  bool `json:"bump_right"`
	RadarLeft  bool `json:"radar_left"`
	RadarRight bool `json:"radar_right"`
	RadarMid   bool `json:"radar_mid"`
	// Ground.state reads true when a sensor SEES floor, not when it detects a
	// drop. Docked, all four read true - which is the robot sitting on the dock
	// plate, not four simultaneous cliffs. So a cliff is a sensor going false.
	Cliff       bool `json:"cliff"`
	GroundCount int  `json:"ground_sensors"`
	GroundOK    int  `json:"ground_seen"`

	// The ToF reads zero and invalid while parked; like the lidar, it is not
	// powered until the robot has a reason to use it.
	TofDist  float64 `json:"tof_distance"`
	TofValid bool    `json:"tof_valid"`

	FrontWall bool `json:"front_wall"`

	// Presence, as carried by THIS record. False here means "not in this
	// record", which is not the same as absent from the robot.
	DustBox  bool `json:"-"`
	WaterBox bool `json:"-"`
	MopLeft  bool `json:"-"`
	MopRight bool `json:"-"`

	SawDustBox  bool `json:"-"`
	SawWaterBox bool `json:"-"`
	SawMopLeft  bool `json:"-"`
	SawMopRight bool `json:"-"`

	// Presence as last reported by the robot, nil until a record carried it.
	DustBoxKnown  *bool `json:"dust_box"`
	WaterBoxKnown *bool `json:"water_box"`
	MopLeftKnown  *bool `json:"mop_holder_left"`
	MopRightKnown *bool `json:"mop_holder_right"`

	// Ultrasound on this robot is carpet detection, not obstacle ranging - the
	// proto only carries a carpet flag, a raw value and comms health.
	Carpet     bool `json:"carpet"`
	CarpetRaw  int  `json:"carpet_raw"`
	UltraError bool `json:"ultrasound_error"`
	Valid      bool `json:"valid"`
}

// packedBools reads a packed `repeated bool`.
func packedBools(b []byte) []bool {
	out := make([]bool, 0, len(b))
	for i := 0; i < len(b); {
		v, ni, ok := pb.ReadVarint(b, i)
		if !ok {
			break
		}
		out = append(out, v != 0)
		i = ni
	}
	return out
}

// boolAt is a bounds-safe index into a packed bool run: the robot does not
// always report every sensor, and a short run should read false rather than
// panic.
func boolAt(vals []bool, i int) bool {
	return i < len(vals) && vals[i]
}

// subSwitch reads a Switch{bool value = 1} submessage.
func subSwitch(b []byte) bool {
	for _, f := range pb.ParseFields(b) {
		if f.Num == 1 && f.Wire == 0 {
			return f.Val != 0
		}
	}
	return false
}

func decodeSensors(rec []byte) *Sensors {
	s := &Sensors{}
	for _, f := range pb.ParseFields(rec) {
		if f.Wire != 2 || f.Num == 1 && len(f.Data) == 0 {
			continue
		}
		switch f.Num {
		case 1: // Ground
			for _, g := range pb.ParseFields(f.Data) {
				if g.Num == 1 && g.Wire == 2 {
					vals := packedBools(g.Data)
					s.GroundCount = len(vals)
					for _, v := range vals {
						if v {
							s.GroundOK++
						}
					}
					s.Cliff = s.GroundCount > 0 && s.GroundOK < s.GroundCount
				}
			}
		case 2: // Bump: BR = 0, BL = 1
			for _, g := range pb.ParseFields(f.Data) {
				if g.Num == 1 && g.Wire == 2 {
					vals := packedBools(g.Data)
					s.BumpRight, s.BumpLeft = boolAt(vals, 0), boolAt(vals, 1)
				}
			}
		case 3: // RadarBump: RBR = 0, RBL = 1, RBM = 2
			for _, g := range pb.ParseFields(f.Data) {
				if g.Num == 1 && g.Wire == 2 {
					vals := packedBools(g.Data)
					s.RadarRight, s.RadarLeft, s.RadarMid =
						boolAt(vals, 0), boolAt(vals, 1), boolAt(vals, 2)
				}
			}
		case 4: // Tof
			for _, g := range pb.ParseFields(f.Data) {
				switch {
				case g.Num == 1 && g.Wire == 5 && len(g.Data) == 4:
					s.TofDist = pb.Float32(g)
				case g.Num == 2 && g.Wire == 0:
					s.TofValid = g.Val == 0 // NORMAL
				}
			}
		case 5:
			s.DustBox, s.SawDustBox = subSwitch(f.Data), true
		case 6:
			s.MopLeft, s.SawMopLeft = subSwitch(f.Data), true
		case 7:
			s.MopRight, s.SawMopRight = subSwitch(f.Data), true
		case 8: // FrontWall
			for _, g := range pb.ParseFields(f.Data) {
				if g.Num == 1 && g.Wire == 0 {
					s.FrontWall = g.Val != 0
				}
			}
		case 9:
			s.WaterBox, s.SawWaterBox = subSwitch(f.Data), true
		case 11: // UltrasoundData
			for _, g := range pb.ParseFields(f.Data) {
				switch {
				case g.Num == 2 && g.Wire == 0:
					s.Carpet = g.Val != 0
				case g.Num == 3 && g.Wire == 0:
					s.UltraError = g.Val != 0
				case g.Num == 4 && g.Wire == 0:
					s.CarpetRaw = int(g.Val)
				}
			}
		}
	}
	if math.IsNaN(s.TofDist) || math.IsInf(s.TofDist, 0) {
		s.TofDist = 0
	}
	s.Valid = true
	return s
}

// WatchSensors follows SensorData so the presence fields are not missed.
//
// They are published once, on change, and the ring is 4 KB - a few dozen
// records - so by the time anything asks the API the record that carried
// "dust box reinserted" is long gone. Sampling on demand catches it only by
// luck; this catches it by being there.
//
// 20 Hz, not 1 Hz. mdsNewest is a shared-memory read, not a subprocess, so it
// is cheap - and at one poll a second the ring turns over between polls and the
// single record carrying the change is simply never seen.
func (l *Local) WatchSensors(stop <-chan struct{}) {
	if !l.Available() {
		return
	}
	tick := time.NewTicker(50 * time.Millisecond)
	defer tick.Stop()
	for {
		if rec, err := mdsNewest("eros.proto.msg.chassis.SensorData"); err == nil {
			l.mergePresence(decodeSensors(rec))
		}
		select {
		case <-stop:
			return
		case <-tick.C:
		}
	}
}

// mergePresence remembers any presence field this record actually carried.
func (l *Local) mergePresence(sn *Sensors) {
	l.mu.Lock()
	defer l.mu.Unlock()
	for _, p := range []struct {
		seen  bool
		val   bool
		cache **bool
	}{
		{sn.SawDustBox, sn.DustBox, &l.dustBox},
		{sn.SawWaterBox, sn.WaterBox, &l.waterBox},
		{sn.SawMopLeft, sn.MopLeft, &l.mopLeft},
		{sn.SawMopRight, sn.MopRight, &l.mopRight},
	} {
		if p.seen {
			v := p.val
			*p.cache = &v
		}
	}
}

// Sensors reports the chassis sensors, accumulated across records.
//
// SensorData is partial, exactly like StatusUpdateNotify: one record carries
// only what changed. Decoding the newest record alone reports every absent
// field as false, which is a claim the robot never made - a docked robot was
// reported with no dust box, no water box and no mops, while the bus record
// two slots back said water_box_state 1.
//
// So the presence fields are remembered once seen, and reported as unknown
// until they are. Everything else stays newest-wins.
func (l *Local) Sensors() (*Sensors, bool) {
	if !l.Available() {
		return nil, false
	}
	rec, err := mdsNewest("eros.proto.msg.chassis.SensorData")
	if err != nil {
		return nil, false
	}
	sn := decodeSensors(rec)
	l.mergePresence(sn)
	l.mu.RLock()
	defer l.mu.RUnlock()
	sn.DustBoxKnown, sn.WaterBoxKnown = l.dustBox, l.waterBox
	sn.MopLeftKnown, sn.MopRightKnown = l.mopLeft, l.mopRight
	return sn, true
}

func (s *server) handleSensors(w http.ResponseWriter, r *http.Request) {
	sn, ok := s.local.Sensors()
	if !ok {
		writeJSON(w, http.StatusOK, map[string]any{"ok": false, "error": "no sensor report"})
		return
	}
	writeJSON(w, http.StatusOK, map[string]any{"ok": true, "sensors": sn})
}

// handleManualMode enters or leaves hand-driving.
//
// Entering forces CleanType to SWEEP_ONLY so the mops stay up: remote control
// is 遥控清扫, remote *clean*, and it drops the mops on whatever floor the robot
// is standing on. Sweep-only still spins the brush - Fan.Suction has no OFF and
// sweep means vacuum - but a dry brush on a hard floor is a different problem
// from a wet mop turning in one spot.
//
// The previous clean type is remembered and put back on exit, so driving the
// robot by hand does not silently reconfigure the next scheduled clean.
// workStatus answers "what is the robot doing" with the same precedence
// /api/status uses: the bus first, because StatusUpdateNotify holds the last
// report regardless of who was listening, and the cloud's cached DP 153 only
// when the bus has nothing.
func (s *server) workStatus() (string, bool) {
	if st, _, ok := s.local.WorkStatusDetail(); ok {
		return st, true
	}
	if st := s.cloud.State().Status; st != "" {
		return st, true
	}
	return "", false
}

// taskRunning is deliberately not the same question as workStatus. The
// reconciled answer says where the robot is, and a robot that drove back to the
// dock to recharge mid-run reads "docked" while its task is still open - seen on
// 2026-08-06, work_status "docked" against work_status_reported "cleaning". For
// the purpose of not ending somebody's clean the raw report wins: if any source
// says cleaning, assume a task is running.
func (s *server) taskRunning() bool {
	if st, ok := s.local.ReportedWorkStatus(); ok && st == "cleaning" {
		return true
	}
	if st, _, ok := s.local.WorkStatusDetail(); ok && st == "cleaning" {
		return true
	}
	return s.cloud.State().Status == "cleaning"
}

func (s *server) handleManualMode(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPost {
		writeJSON(w, http.StatusMethodNotAllowed, map[string]string{"error": "POST only"})
		return
	}
	ctx, cancel := context.WithTimeout(r.Context(), 20*time.Second)
	defer cancel()

	on := r.URL.Query().Get("on") != "0"
	force := r.URL.Query().Get("force") == "1"
	sweepOnly := 0

	if on {
		// Entering is not a passive act. It writes CleanParam immediately and
		// sends STOP_TASK on the way out, and either one ends a run that is
		// already going. The page enters on a tab switch alone, so opening it
		// mid-clean stopped the robot. Refuse unless the caller has been asked
		// and said yes.
		if !force && s.taskRunning() {
			st, _ := s.workStatus()
			writeJSON(w, http.StatusOK, map[string]any{
				"ok": false, "busy": true, "work_status": st,
			})
			return
		}
		if cur, err := readSettings(); err == nil && cur.CleanType != nil {
			s.manualMu.Lock()
			if s.manualPrev == nil {
				prev := *cur.CleanType
				s.manualPrev = &prev
			}
			s.manualMu.Unlock()
		}
		if err := s.local.SetCleanParams(ctx, &CleanSettings{CleanType: &sweepOnly}); err != nil {
			writeJSON(w, http.StatusOK, map[string]any{"ok": false, "error": err.Error()})
			return
		}
		writeJSON(w, http.StatusOK, map[string]any{"ok": true, "manual": true, "clean_type": "sweep_only"})
		return
	}

	// Leaving: stop, park the mops, and put the clean type back.
	_ = s.brakeAndPark(ctx)
	s.manualMu.Lock()
	prev := s.manualPrev
	s.manualPrev = nil
	s.manualMu.Unlock()
	restored := "unchanged"
	if prev != nil {
		if err := s.local.SetCleanParams(ctx, &CleanSettings{CleanType: prev}); err == nil {
			restored = nameOf(typeNames, prev)
		}
	}
	writeJSON(w, http.StatusOK, map[string]any{"ok": true, "manual": false, "clean_type": restored})
}
