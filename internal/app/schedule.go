package app

// Cleaning schedules, read from the robot's own config.
//
// DP 164 carries these over the cloud channel as an opaque Raw blob we never
// decoded. On the device they are plain files, one per schedule, in the same
// OpenCV FileStorage dialect as the map metadata - so this is another case
// where the local copy is both easier to read and always present.
//
// Reads only. Writes go through app.TimerRequest - see timer.go - because the
// running stack caches these files and a write would need a reload the format
// does not document.
//
// The room selection lives here too, which saves a round trip: a rooms schedule
// stores its order as `rooms_order` and its per-room parameters as a run of
// `rooms_custom_N` blocks.
//
//	rooms_order:
//	   room5: 0
//	   room1: 1
//	   room7: 2
//	rooms_custom_0:
//	   room: 5
//	   room_order: 0

import (
	"bufio"
	"fmt"
	"net/http"
	"os"
	"path/filepath"
	"regexp"
	"sort"
	"strconv"
	"strings"
)

// var, not const, so the tests can point them at a temp dir.
var (
	scheduleDir        = "/userdata/user_config/schedule_clean"
	scheduleQuarantine = "/userdata/user_config/schedule_backups"
)

// scheduleSlotFile matches the only names that belong in scheduleDir.
// robotapp arms every file it finds there, so a backup copy left beside the
// slots - schedule_clean_2.yaml.pre-timertest-20260817 - runs as a real timer
// on whatever days that old record carried.
var scheduleSlotFile = regexp.MustCompile(`^schedule_clean_[0-9]+\.yaml$`)

type Schedule struct {
	ID       int      `json:"id"`
	Enabled  bool     `json:"enabled"`
	Valid    bool     `json:"valid"`
	Hour     int      `json:"hour"`
	Minute   int      `json:"minute"`
	At       string   `json:"at"`
	Days     []string `json:"days"`
	WeekBits int      `json:"week_bits"`
	Mode     int      `json:"mode"`
	MapID    int      `json:"map_id"`
	Rooms    []int    `json:"rooms"` // room ids, in cleaning order
	Suction  int      `json:"suction"`
	Water    int      `json:"mop_water"`
	File     string   `json:"file"`
}

// weekdays is indexed by bit position. Bit 0 is Sunday: the two live schedules
// are 18 (0b0010010) and 36 (0b0100100), which read as Mon+Thu and Tue+Fri -
// consistent with a Sunday-first week and with a robot that cleans twice a week.
// Not confirmed against the app.
var weekdays = []string{"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"}

func daysFromBits(bits int) []string {
	var out []string
	for i, d := range weekdays {
		if bits&(1<<i) != 0 {
			out = append(out, d)
		}
	}
	return out
}

func readSchedules() ([]Schedule, error) {
	entries, err := os.ReadDir(scheduleDir)
	if err != nil {
		return nil, err
	}
	var out []Schedule
	for _, e := range entries {
		if e.IsDir() || !scheduleSlotFile.MatchString(e.Name()) {
			continue
		}
		p := filepath.Join(scheduleDir, e.Name())
		id, err := scanInt(p, "schedule_id")
		if err != nil {
			continue
		}
		opened, _ := scanInt(p, "status_opened")
		valid, _ := scanInt(p, "status_valid")
		// status_valid 0 is an empty slot, not a schedule. The robot writes
		// these out as all-zero records and ignores them; listing them shows a
		// phantom "#0 00:00" with no days and no rooms.
		if valid == 0 {
			continue
		}
		hour, _ := scanInt(p, "desc_timing_hours")
		min, _ := scanInt(p, "desc_timing_minutes")
		bits, _ := scanInt(p, "desc_cycle_week_bits")
		mode, _ := scanInt(p, "action_schedule_mode")
		mapID, _ := scanInt(p, "action_schedule_map_id")
		rooms := readScheduleRooms(p)
		suction, _ := scanInt(p, "action_schedule_suction")
		water, _ := scanInt(p, "action_schedule_mop_water")
		out = append(out, Schedule{
			ID: id, Enabled: opened != 0, Valid: valid != 0,
			Hour: hour, Minute: min, At: fmt.Sprintf("%02d:%02d", hour, min),
			Days: daysFromBits(bits), WeekBits: bits,
			Mode: mode, Suction: suction, Water: water, File: e.Name(),
			MapID: mapID, Rooms: rooms,
		})
	}
	sort.Slice(out, func(i, j int) bool { return out[i].ID < out[j].ID })
	return out, nil
}

// strayScheduleFiles lists everything in scheduleDir that is not a slot file.
// Each one is an extra armed timer, so this is a fault list, not a listing.
func strayScheduleFiles() ([]string, error) {
	entries, err := os.ReadDir(scheduleDir)
	if err != nil {
		return nil, err
	}
	var out []string
	for _, e := range entries {
		if e.IsDir() || scheduleSlotFile.MatchString(e.Name()) {
			continue
		}
		out = append(out, e.Name())
	}
	sort.Strings(out)
	return out, nil
}

// quarantineStraySchedules moves strays into scheduleQuarantine.
// robotapp reads the directory once at boot, so a stray it already loaded
// keeps firing until the next reboot - the move only fixes the boot after.
func quarantineStraySchedules() ([]string, error) {
	strays, err := strayScheduleFiles()
	if err != nil || len(strays) == 0 {
		return nil, err
	}
	if err := os.MkdirAll(scheduleQuarantine, 0o700); err != nil {
		return nil, err
	}
	var moved []string
	for _, name := range strays {
		dst := filepath.Join(scheduleQuarantine, name)
		if err := os.Rename(filepath.Join(scheduleDir, name), dst); err != nil {
			return moved, err
		}
		moved = append(moved, name)
	}
	return moved, nil
}

// handleSchedules serves the YAML. Disk is the fresher of the two sources: the
// robot rewrites it on every accepted edit, INQUIRY answers the boot snapshot.
func (s *server) handleSchedules(w http.ResponseWriter, r *http.Request) {
	sch, err := readSchedules()
	if err != nil {
		writeJSON(w, http.StatusServiceUnavailable, map[string]any{
			"ok": false, "error": fmt.Sprintf("no schedules on disk: %v", err),
		})
		return
	}
	out := map[string]any{"ok": true, "schedules": sch}
	if strays, err := strayScheduleFiles(); err == nil && len(strays) > 0 {
		out["strays"] = strays
		out["warning"] = fmt.Sprintf(
			"%d stray file(s) in %s are armed as extra timers until the next reboot",
			len(strays), scheduleDir)
	}
	writeJSON(w, http.StatusOK, out)
}

// readScheduleRooms returns the schedule's rooms in cleaning order.
//
// The file gives the order as a map of room -> position rather than a list, so
// it has to be sorted back into one. Positions are zero-based and contiguous in
// every schedule seen, but sorting by position rather than trusting that costs
// nothing.
func readScheduleRooms(path string) []int {
	f, err := os.Open(path)
	if err != nil {
		return nil
	}
	defer f.Close()

	type slot struct{ id, order int }
	var slots []slot
	inOrder := false
	sc := bufio.NewScanner(f)
	for sc.Scan() {
		line := sc.Text()
		trimmed := strings.TrimSpace(line)
		if trimmed == "" {
			continue
		}
		if line == trimmed {
			// A new top-level key ends the rooms_order block.
			inOrder = trimmed == "rooms_order:"
			continue
		}
		if !inOrder {
			continue
		}
		key, val, ok := strings.Cut(trimmed, ":")
		if !ok || !strings.HasPrefix(key, "room") {
			continue
		}
		id, err1 := strconv.Atoi(strings.TrimPrefix(strings.TrimSpace(key), "room"))
		order, err2 := strconv.Atoi(strings.TrimSpace(val))
		if err1 != nil || err2 != nil {
			continue
		}
		slots = append(slots, slot{id: id, order: order})
	}
	sort.Slice(slots, func(i, j int) bool { return slots[i].order < slots[j].order })
	out := make([]int, 0, len(slots))
	for _, s := range slots {
		out = append(out, s.id)
	}
	return out
}

// ScheduleRoom is one room in a schedule, with the parameters it will be
// cleaned with. Mirrors a rooms_custom_N block.
type ScheduleRoom struct {
	ID          int `json:"id"`
	Order       int `json:"order"`
	CleanType   int `json:"clean_type"`
	Fan         int `json:"fan"`
	MopWater    int `json:"mop_water"`
	CornerClean int `json:"corner_clean"`
	CleanExtent int `json:"clean_extent"`
	CleanTimes  int `json:"clean_times"`
}

// ScheduleFull is everything MOTIFY needs to send back.
//
// The whole record has to be rebuilt for an edit, because MOTIFY replaces
// rather than merges - so anything not read here is anything silently dropped.
type ScheduleFull struct {
	ID         int            `json:"id"`
	Valid      bool           `json:"valid"`
	Opened     bool           `json:"opened"`
	IgnoreOnce bool           `json:"ignore_once"`
	Trigger    int            `json:"trigger"`
	UserTZ     int            `json:"user_tz"`
	Summer     bool           `json:"summer"`
	Hour       int            `json:"hour"`
	Minute     int            `json:"minute"`
	WeekBits   int            `json:"week_bits"`
	ActionType int            `json:"action_type"`
	ActionMode int            `json:"action_mode"`
	MapID      int            `json:"map_id"`
	SmartMode  bool           `json:"smart_mode"`
	SceneID    int            `json:"scene_id"`
	Rooms      []ScheduleRoom `json:"rooms"`
	File       string         `json:"file"`

	// Addition (TimerInfo field 4). MOTIFY replaces the record, so dropping
	// these blanks the robot's own identity for the schedule.
	CreateTime uint64 `json:"create_time"`
	CreateUser string `json:"create_user_id"`
	RenewTime  uint64 `json:"renew_time"`
	RenewUser  string `json:"renew_user_id"`
}

// readScheduleFull reads one schedule file in full.
func readScheduleFull(id int) (*ScheduleFull, error) {
	entries, err := os.ReadDir(scheduleDir)
	if err != nil {
		return nil, err
	}
	for _, e := range entries {
		if e.IsDir() || !scheduleSlotFile.MatchString(e.Name()) {
			continue
		}
		p := filepath.Join(scheduleDir, e.Name())
		got, err := scanInt(p, "schedule_id")
		if err != nil || got != id {
			continue
		}
		s := &ScheduleFull{ID: id, File: e.Name()}
		geti := func(k string) int { v, _ := scanInt(p, k); return v }
		s.Valid = geti("status_valid") != 0
		s.Opened = geti("status_opened") != 0
		s.IgnoreOnce = geti("status_ignore_once") != 0
		s.Trigger = geti("desc_trigger")
		s.UserTZ = geti("desc_timing_user_tz")
		s.Summer = geti("desc_timing_summer") != 0
		s.Hour = geti("desc_timing_hours")
		s.Minute = geti("desc_timing_minutes")
		s.WeekBits = geti("desc_cycle_week_bits")
		s.ActionType = geti("action_schedule_type")
		s.ActionMode = geti("action_schedule_mode")
		s.MapID = geti("action_schedule_map_id")
		s.SceneID = geti("action_schedule_scene_id")
		s.SmartMode = geti("smart_mode_sw") != 0
		getu := func(k string) uint64 {
			v, ok := scanKey(p, k)
			if !ok {
				return 0
			}
			n, _ := strconv.ParseUint(strings.Trim(v, `"`), 10, 64)
			return n
		}
		gets := func(k string) string {
			v, _ := scanKey(p, k)
			return strings.Trim(v, `"`)
		}
		s.CreateTime = getu("addition_create_time")
		s.CreateUser = gets("addition_create_user_id")
		s.RenewTime = getu("addition_renew_time")
		s.RenewUser = gets("addition_renew_user_id")
		s.Rooms = readScheduleRoomBlocks(p)
		return s, nil
	}
	return nil, fmt.Errorf("no schedule %d", id)
}

// readScheduleRoomBlocks parses the rooms_custom_N blocks.
func readScheduleRoomBlocks(path string) []ScheduleRoom {
	f, err := os.Open(path)
	if err != nil {
		return nil
	}
	defer f.Close()

	var out []ScheduleRoom
	var cur *ScheduleRoom
	flush := func() {
		if cur != nil {
			out = append(out, *cur)
			cur = nil
		}
	}
	sc := bufio.NewScanner(f)
	for sc.Scan() {
		line := sc.Text()
		trimmed := strings.TrimSpace(line)
		if trimmed == "" {
			continue
		}
		if line == trimmed {
			flush()
			if strings.HasPrefix(trimmed, "rooms_custom_") {
				cur = &ScheduleRoom{CleanTimes: 1}
			}
			continue
		}
		if cur == nil {
			continue
		}
		key, val, ok := strings.Cut(trimmed, ":")
		if !ok {
			continue
		}
		n, err := strconv.Atoi(strings.TrimSpace(val))
		if err != nil {
			continue
		}
		switch strings.TrimSpace(key) {
		case "room":
			cur.ID = n
		case "room_order":
			cur.Order = n
		case "clean_type":
			cur.CleanType = n
		case "fan_suction":
			cur.Fan = n
		case "mop_water":
			cur.MopWater = n
		case "corner_clean":
			cur.CornerClean = n
		case "clean_extent":
			cur.CleanExtent = n
		case "clean_times":
			cur.CleanTimes = n
		}
	}
	flush()
	sort.Slice(out, func(i, j int) bool { return out[i].Order < out[j].Order })
	return out
}
