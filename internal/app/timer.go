package app

// Schedule editing.
//
// schedule.go reads the robot's own YAML and that stays the display path: it is
// the persisted truth and needs no round trip. This is the write half, and it
// does not touch those files. Dropping a rewritten YAML in place would not work
// anyway - the running stack caches them - so changes go through the request
// API the app uses.
//
//	app.TimerRequest { Header header = 1; TimerRequest req = 2 }
//	TimerRequest     { Method method = 1; uint32 seq = 2; TimerInfo timer = 3 }
//	  Method: DEFAULT=0 ADD=1 DELETE=2 MOTIFY=3 OPEN=4 CLOSE=5 INQUIRY=6 IGNORE_ONCE=7
//	TimerInfo        { Id id = 1; Status status = 2; Desc desc = 3;
//	                   Addition addition = 4; Action action = 5 }
//	  Id     { uint32 value = 1 }
//	  Status { bool valid = 1; bool opened = 2; bool ignore_once = 3 }
//	  Desc   { Trigger trigger = 1; Timing timing = 2; Cycle cycle = 3 }
//	  Timing { uint32 user_tz = 1; bool summer = 2; uint32 hours = 3; uint32 minutes = 4 }
//	  Cycle  { uint32 week_bits = 1 }
//
// MOTIFY is spelled that way upstream.
//
// The channel does not appear in channel_config.yaml and does not exist until
// something publishes it; cloud_cli then creates it, 16384+112 like any
// unlisted channel. Confirmed on hardware: publishing INQUIRY directly moved
// app.TimerResponse by the same 405 bytes the MQTT path produces, with no
// `mqtt recv` line in the log.

import (
	"context"
	"fmt"
	"github.com/dhumpf/x10-ctl/internal/pb"
	"net/http"
	"sort"
	"strings"
	"time"
)

const mdsTimerRequest = "eros.proto.msg.app.TimerRequest"

const (
	timerAdd uint64 = iota + 1
	timerDelete
	timerModify
	timerOpen
	timerClose
	timerInquiry
	timerIgnoreOnce
)

var timerMethods = map[string]uint64{
	"enable": timerOpen, "open": timerOpen,
	"disable": timerClose, "close": timerClose,
	"delete": timerDelete,
	"skip":   timerIgnoreOnce, "ignore_once": timerIgnoreOnce,
	"inquiry": timerInquiry,
	"retime":  timerModify, "modify": timerModify,
}

// Timer publishes one TimerRequest.
//
// seq shares the counter the mode commands use, so a reply can be matched
// against the request that caused it without a second numbering scheme.
func (l *Local) Timer(ctx context.Context, method, seq uint64, timer []byte) error {
	req := append(pb.FieldVarint(1, method), pb.FieldVarint(2, seq)...)
	if len(timer) > 0 {
		req = append(req, pb.FieldBytes(3, timer)...)
	}
	return l.mdsPublish(ctx, mdsTimerRequest, pb.FieldBytes(2, req))
}

// timerByID is the minimal TimerInfo: just an Id. Enough for OPEN, CLOSE,
// DELETE and IGNORE_ONCE, which act on an existing schedule.
func timerByID(id uint64) []byte {
	return pb.FieldBytes(1, pb.FieldVarint(1, id))
}

// scheduleFull returns the record a MOTIFY is rebuilt from.
//
// Disk first: the robot rewrites the YAML on every accepted edit, while
// INQUIRY keeps answering the snapshot it booted with. Rebuilding from the bus
// copy silently reverts whatever the previous edit changed.
func (s *server) scheduleFull(ctx context.Context, id int) (*ScheduleFull, error) {
	full, err := readScheduleFull(id)
	if err == nil {
		return full, nil
	}
	if s.local.Available() {
		c, cancel := context.WithTimeout(ctx, 6*time.Second)
		defer cancel()
		if all, ierr := s.local.InquireSchedules(c, s.nextSeq()); ierr == nil {
			for _, sc := range all {
				if sc.ID == id {
					return sc, nil
				}
			}
		}
	}
	return nil, err
}

func (s *server) handleSchedule(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPost {
		writeJSON(w, http.StatusMethodNotAllowed, map[string]string{"error": "POST only"})
		return
	}
	var body struct {
		ID       uint64 `json:"id"`
		Action   string `json:"action"`
		Hour     int    `json:"hour"`
		Minute   int    `json:"minute"`
		WeekBits int    `json:"week_bits"`
	}
	if err := decodeJSON(w, r, &body); err != nil {
		writeJSON(w, http.StatusBadRequest, map[string]any{"ok": false, "error": "bad json"})
		return
	}
	method, ok := timerMethods[strings.ToLower(strings.TrimSpace(body.Action))]
	if !ok {
		writeJSON(w, http.StatusBadRequest, map[string]any{
			"ok": false, "error": "action must be enable, disable, delete, skip, retime or inquiry",
		})
		return
	}
	if method != timerInquiry && body.ID == 0 {
		writeJSON(w, http.StatusBadRequest, map[string]any{"ok": false, "error": "id is required"})
		return
	}

	var timer []byte
	switch method {
	case timerInquiry:
	case timerModify:
		if body.Hour < 0 || body.Hour > 23 || body.Minute < 0 || body.Minute > 59 {
			writeJSON(w, http.StatusBadRequest, map[string]any{
				"ok": false, "error": "hour must be 0..23 and minute 0..59",
			})
			return
		}
		if body.WeekBits < 0 || body.WeekBits > 0x7f {
			writeJSON(w, http.StatusBadRequest, map[string]any{
				"ok": false, "error": "week_bits must be 0..127, bit 0 is Sunday",
			})
			return
		}
		// MOTIFY replaces the whole record; a partial Id+Desc frame is
		// silently dropped by the robot. Rebuild from the live record.
		cur, err := s.scheduleFull(r.Context(), int(body.ID))
		if err != nil {
			writeJSON(w, http.StatusBadRequest, map[string]any{"ok": false, "error": err.Error()})
			return
		}
		cur.Hour, cur.Minute, cur.WeekBits = body.Hour, body.Minute, body.WeekBits
		cur.Trigger = 0
		if body.WeekBits != 0 {
			cur.Trigger = 1
		}
		timer = encodeTimerInfo(cur)
	default:
		timer = timerByID(body.ID)
	}

	ctx, cancel := context.WithTimeout(r.Context(), 15*time.Second)
	defer cancel()

	if err := s.local.Timer(ctx, method, s.nextSeq(), timer); err != nil {
		writeJSON(w, http.StatusOK, map[string]any{"ok": false, "error": err.Error()})
		return
	}

	// The robot rewrites its schedule YAML as part of handling this, so a short
	// pause makes the reply reflect the change rather than the state before it.
	select {
	case <-ctx.Done():
	case <-time.After(700 * time.Millisecond):
	}
	sch, err := readSchedules()
	out := map[string]any{"ok": true, "action": body.Action, "id": body.ID, "bus": "local"}
	if err == nil {
		out["schedules"] = sch
	} else {
		out["warning"] = fmt.Sprintf("sent, but could not re-read schedules: %v", err)
	}
	writeJSON(w, http.StatusOK, out)
}

// encodeTimerInfo rebuilds a complete TimerInfo for MOTIFY.
//
// MOTIFY replaces the record rather than merging into it, so every field the
// robot should keep has to be sent back. Anything omitted here is a setting
// silently lost on the next edit, which is why ScheduleFull reads the whole
// file rather than the handful of fields the list view shows.
//
//	TimerInfo { Id id = 1; Status status = 2; Desc desc = 3; Action action = 5 }
//	  Status { bool valid = 1; bool opened = 2; bool ignore_once = 3 }
//	  Desc   { Trigger trigger = 1; Timing timing = 2; Cycle cycle = 3 }
//	  Timing { uint32 user_tz = 1; bool summer = 2;
//	           uint32 hours = 3; uint32 minutes = 4 }
//	  Action { Type type = 1; ScheduleRoomsClean sche_rooms_clean = 4 }
//	  ScheduleRoomsClean { Mode mode = 1; General general = 2; Custom custom = 3 }
//	  Custom { uint32 map_id = 1; Switch smart_mode_sw = 2;
//	           repeated Room rooms = 3 }
//	  Custom.Room { uint32 id = 1; uint32 order = 2; CleanType clean_type = 6;
//	                Fan fan = 7; MopMode mop_mode = 8;
//	                CleanExtent clean_extent = 9; uint32 clean_times = 10 }
//
// addition (field 4) is deliberately not sent: it carries create/renew times
// and user ids the robot maintains itself.
func encodeTimerInfo(s *ScheduleFull) []byte {
	boolField := func(num uint64, b bool) []byte {
		if !b {
			return nil // proto3 omits false
		}
		return pb.FieldVarint(num, 1)
	}
	// An enum wrapped in its own submessage, the shape used throughout these
	// protos: present-but-empty means the zero value, absent means unset.
	enumMsg := func(num uint64, v int) []byte {
		var inner []byte
		if v != 0 {
			inner = pb.FieldVarint(1, uint64(v))
		}
		return pb.FieldBytes(num, inner)
	}

	info := pb.FieldBytes(1, pb.FieldVarint(1, uint64(s.ID)))

	status := boolField(1, s.Valid)
	status = append(status, boolField(2, s.Opened)...)
	status = append(status, boolField(3, s.IgnoreOnce)...)
	info = append(info, pb.FieldBytes(2, status)...)

	timing := pb.FieldVarint(1, uint64(s.UserTZ))
	timing = append(timing, boolField(2, s.Summer)...)
	timing = append(timing, pb.FieldVarint(3, uint64(s.Hour))...)
	timing = append(timing, pb.FieldVarint(4, uint64(s.Minute))...)
	desc := pb.FieldVarint(1, uint64(s.Trigger))
	desc = append(desc, pb.FieldBytes(2, timing)...)
	desc = append(desc, pb.FieldBytes(3, pb.FieldVarint(1, uint64(s.WeekBits)))...)
	info = append(info, pb.FieldBytes(3, desc)...)

	// Addition. Omitting it blanked create_time/renew_time on every edit, and
	// the robot then persisted the record as a fresh slot 0 instead of in place.
	now := uint64(time.Now().UnixMilli())
	create := s.CreateTime
	if create == 0 {
		create = now
	}
	addition := pb.FieldVarint(1, create)
	if s.CreateUser != "" {
		addition = append(addition, pb.FieldBytes(2, []byte(s.CreateUser))...)
	}
	addition = append(addition, pb.FieldVarint(3, now)...)
	if s.RenewUser != "" {
		addition = append(addition, pb.FieldBytes(4, []byte(s.RenewUser))...)
	}
	info = append(info, pb.FieldBytes(4, addition)...)

	var rooms []byte
	for i, r := range s.Rooms {
		order := r.Order
		if order == 0 && i > 0 {
			order = i // keep a sane order if the caller only gave ids
		}
		room := pb.FieldVarint(1, uint64(r.ID))
		if order != 0 {
			room = append(room, pb.FieldVarint(2, uint64(order))...)
		}
		room = append(room, enumMsg(6, r.CleanType)...)
		room = append(room, enumMsg(7, r.Fan)...)
		// MopMode carries level and corner_clean together.
		var mop []byte
		if r.MopWater != 0 {
			mop = pb.FieldVarint(1, uint64(r.MopWater))
		}
		if r.CornerClean != 0 {
			mop = append(mop, pb.FieldVarint(2, uint64(r.CornerClean))...)
		}
		room = append(room, pb.FieldBytes(8, mop)...)
		room = append(room, enumMsg(9, r.CleanExtent)...)
		if r.CleanTimes > 0 {
			room = append(room, pb.FieldVarint(10, uint64(r.CleanTimes))...)
		}
		rooms = append(rooms, pb.FieldBytes(3, room)...)
	}

	custom := pb.FieldVarint(1, uint64(s.MapID))
	custom = append(custom, pb.FieldBytes(2, boolField(1, s.SmartMode))...)
	custom = append(custom, rooms...)

	sche := pb.FieldVarint(1, uint64(s.ActionMode))
	sche = append(sche, pb.FieldBytes(3, custom)...)

	action := pb.FieldVarint(1, uint64(s.ActionType))
	action = append(action, pb.FieldBytes(4, sche)...)
	return append(info, pb.FieldBytes(5, action)...)
}

// handleScheduleRooms replaces which rooms a schedule cleans, and in what order.
func (s *server) handleScheduleRooms(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPost {
		writeJSON(w, http.StatusMethodNotAllowed, map[string]string{"error": "POST only"})
		return
	}
	var body struct {
		ID    int            `json:"id"`
		Rooms []ScheduleRoom `json:"rooms"`
	}
	if err := decodeJSON(w, r, &body); err != nil {
		writeJSON(w, http.StatusBadRequest, map[string]any{"ok": false, "error": "bad json"})
		return
	}
	if body.ID <= 0 || len(body.Rooms) == 0 {
		writeJSON(w, http.StatusBadRequest, map[string]any{
			"ok": false, "error": "id and at least one room are required",
		})
		return
	}

	cur, err := s.scheduleFull(r.Context(), body.ID)
	if err != nil {
		writeJSON(w, http.StatusBadRequest, map[string]any{"ok": false, "error": err.Error()})
		return
	}
	known, err := readRoomDetails()
	if err != nil {
		writeJSON(w, http.StatusServiceUnavailable, map[string]any{"ok": false, "error": err.Error()})
		return
	}
	have := map[int]bool{}
	for _, rm := range known.Rooms {
		have[rm.ID] = true
	}

	// Carry the existing per-room parameters over for rooms already in the
	// schedule; a caller that only reorders should not have to restate them.
	prev := map[int]ScheduleRoom{}
	for _, rm := range cur.Rooms {
		prev[rm.ID] = rm
	}
	out := make([]ScheduleRoom, 0, len(body.Rooms))
	for i, rm := range body.Rooms {
		if !have[rm.ID] {
			writeJSON(w, http.StatusBadRequest, map[string]any{
				"ok": false, "error": fmt.Sprintf("no room %d on this map", rm.ID),
			})
			return
		}
		merged := rm
		if old, ok := prev[rm.ID]; ok && rm.CleanTimes == 0 {
			merged = old
		}
		merged.ID, merged.Order = rm.ID, i
		if merged.CleanTimes == 0 {
			merged.CleanTimes = 1
		}
		out = append(out, merged)
	}
	cur.Rooms = out

	ctx, cancel := context.WithTimeout(r.Context(), 20*time.Second)
	defer cancel()
	if err := s.local.Timer(ctx, timerModify, s.nextSeq(), encodeTimerInfo(cur)); err != nil {
		writeJSON(w, http.StatusOK, map[string]any{"ok": false, "error": err.Error()})
		return
	}
	// Read back off disk once the robot has had time to rewrite the file. The
	// bus answers the boot snapshot, which would hide a successful edit.
	select {
	case <-ctx.Done():
	case <-time.After(700 * time.Millisecond):
	}
	after, err := readScheduleFull(body.ID)
	if err != nil {
		after = nil
	}
	writeJSON(w, http.StatusOK, map[string]any{"ok": true, "bus": "local", "schedule": after})
}

// --- reading schedules back off the bus
//
// INQUIRY answers the snapshot the robot booted with, and that snapshot does
// not move: on 2026-08-20 it was still serving the July records while every
// edit since had landed in schedule_clean with a fresh renew_time. Since the
// Addition fix an accepted MOTIFY rewrites the YAML, so disk is the readback
// path and this is kept only as the fallback when a file cannot be read.

// InquireSchedules asks the robot for its schedules and decodes the reply.
func (l *Local) InquireSchedules(ctx context.Context, seq uint64) ([]*ScheduleFull, error) {
	if err := l.Timer(ctx, timerInquiry, seq, nil); err != nil {
		return nil, err
	}
	deadline := time.Now().Add(4 * time.Second)
	for time.Now().Before(deadline) {
		if rec, err := mdsNewest(mdsTimerResponse); err == nil {
			if got, ok := parseTimerResponse(rec, seq); ok {
				return got, nil
			}
		}
		select {
		case <-ctx.Done():
			return nil, ctx.Err()
		case <-time.After(150 * time.Millisecond):
		}
	}
	return nil, fmt.Errorf("no TimerResponse for seq %d", seq)
}

const mdsTimerResponse = "eros.proto.msg.app.TimerResponse"

// enumOf reads the {value = 1} submessage these protos wrap every enum in.
// Absent and present-but-empty both mean zero.
func enumOf(b []byte) int {
	for _, f := range pb.ParseFields(b) {
		if f.Num == 1 && f.Wire == 0 {
			return int(f.Val)
		}
	}
	return 0
}

// parseTimerResponse decodes TimerResponse, but only for our own sequence -
// the channel holds whatever was last published, which may be a reply to the
// app rather than to us.
func parseTimerResponse(rec []byte, wantSeq uint64) ([]*ScheduleFull, bool) {
	for _, f := range pb.ParseFields(rec) {
		if f.Num != 2 || f.Wire != 2 {
			continue
		}
		var seq uint64
		var haveSeq bool
		var out []*ScheduleFull
		for _, g := range pb.ParseFields(f.Data) {
			switch {
			case g.Num == 2 && g.Wire == 0:
				seq, haveSeq = g.Val, true
			case g.Num == 4 && g.Wire == 2:
				if s := parseTimerInfo(g.Data); s != nil {
					out = append(out, s)
				}
			}
		}
		if haveSeq && seq == wantSeq {
			return out, true
		}
	}
	return nil, false
}

func parseTimerInfo(b []byte) *ScheduleFull {
	s := &ScheduleFull{}
	for _, f := range pb.ParseFields(b) {
		if f.Wire != 2 {
			continue
		}
		switch f.Num {
		case 1: // Id
			s.ID = enumOf(f.Data)
		case 2: // Status
			for _, g := range pb.ParseFields(f.Data) {
				switch g.Num {
				case 1:
					s.Valid = g.Val != 0
				case 2:
					s.Opened = g.Val != 0
				case 3:
					s.IgnoreOnce = g.Val != 0
				}
			}
		case 4: // Addition
			for _, g := range pb.ParseFields(f.Data) {
				switch {
				case g.Num == 1 && g.Wire == 0:
					s.CreateTime = g.Val
				case g.Num == 2 && g.Wire == 2:
					s.CreateUser = string(g.Data)
				case g.Num == 3 && g.Wire == 0:
					s.RenewTime = g.Val
				case g.Num == 4 && g.Wire == 2:
					s.RenewUser = string(g.Data)
				}
			}
		case 3: // Desc
			for _, g := range pb.ParseFields(f.Data) {
				switch {
				case g.Num == 1 && g.Wire == 0:
					s.Trigger = int(g.Val)
				case g.Num == 2 && g.Wire == 2:
					for _, h := range pb.ParseFields(g.Data) {
						switch h.Num {
						case 1:
							s.UserTZ = int(h.Val)
						case 2:
							s.Summer = h.Val != 0
						case 3:
							s.Hour = int(h.Val)
						case 4:
							s.Minute = int(h.Val)
						}
					}
				case g.Num == 3 && g.Wire == 2:
					s.WeekBits = enumOf(g.Data)
				}
			}
		case 5: // Action
			parseTimerAction(f.Data, s)
		}
	}
	if s.ID == 0 {
		return nil
	}
	return s
}

func parseTimerAction(b []byte, s *ScheduleFull) {
	for _, f := range pb.ParseFields(b) {
		switch {
		case f.Num == 1 && f.Wire == 0:
			s.ActionType = int(f.Val)
		case f.Num == 4 && f.Wire == 2: // sche_rooms_clean
			for _, g := range pb.ParseFields(f.Data) {
				switch {
				case g.Num == 1 && g.Wire == 0:
					s.ActionMode = int(g.Val)
				case g.Num == 3 && g.Wire == 2: // Custom
					parseTimerCustom(g.Data, s)
				}
			}
		}
	}
}

func parseTimerCustom(b []byte, s *ScheduleFull) {
	for _, f := range pb.ParseFields(b) {
		switch {
		case f.Num == 1 && f.Wire == 0:
			s.MapID = int(f.Val)
		case f.Num == 2 && f.Wire == 2:
			s.SmartMode = enumOf(f.Data) != 0
		case f.Num == 3 && f.Wire == 2: // Room
			r := ScheduleRoom{CleanTimes: 1}
			for _, g := range pb.ParseFields(f.Data) {
				switch {
				case g.Num == 1 && g.Wire == 0:
					r.ID = int(g.Val)
				case g.Num == 2 && g.Wire == 0:
					r.Order = int(g.Val)
				case g.Num == 6 && g.Wire == 2:
					r.CleanType = enumOf(g.Data)
				case g.Num == 7 && g.Wire == 2:
					r.Fan = enumOf(g.Data)
				case g.Num == 8 && g.Wire == 2: // MopMode: level, corner_clean
					for _, h := range pb.ParseFields(g.Data) {
						switch h.Num {
						case 1:
							r.MopWater = int(h.Val)
						case 2:
							r.CornerClean = int(h.Val)
						}
					}
				case g.Num == 9 && g.Wire == 2:
					r.CleanExtent = enumOf(g.Data)
				case g.Num == 10 && g.Wire == 0:
					r.CleanTimes = int(g.Val)
				}
			}
			if r.ID > 0 {
				s.Rooms = append(s.Rooms, r)
			}
		}
	}
	sort.Slice(s.Rooms, func(i, j int) bool { return s.Rooms[i].Order < s.Rooms[j].Order })
}
