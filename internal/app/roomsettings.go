package app

// Per-room cleaning settings and room order.
//
// Two different mechanisms sit behind "clean this room like that", and the
// Chinese comments in the recovered protos are what separate them:
//
//	通用房间清扫，清洁参数走 clean_param 中 area_clean_param 参数
//	定制房间清扫，设置每个房间不同的清扫参数走 stream 定制参数
//	（通过 MapEditRequest.RoomsCustom 设置）
//
// General room cleans take one set of parameters for every room, from
// area_clean_param. Per-room parameters are a separate store, written through
// MapEditRequest.RoomsCustom, and SelectRoomsClean only selects between the two
// with its mode field. So changing "the settings for the study" is a map edit,
// not a clean-parameter change, which is why it was not reachable from
// settings.go.
//
//	MapEditRequest { Header header = 1; MapEditRequest req = 2 }
//	  Method method = 1;   uint32 seq = 2;   uint32 map_id = 3;
//	  Exclusive exclusive = 4;   RoomsCustom rooms_custom = 10
//
// method, seq and map_id are not optional. Without them the robot replies
// MapEditResponse{result: FAILED} and changes nothing, while mds_pub still
// reports success - so an edit that never happened looks exactly like one that
// did unless the response is read.
//	RoomsCustom {
//	  Enable custom_enable = 1;   // { bool value = 1 } - the master switch
//	  Order  rooms_order   = 2;   // { repeated Room{id=1, order=2} rooms = 1 }
//	  Parm   rooms_parm    = 3;   // { repeated Room{id=1, Custom custom=2} rooms = 1 }
//	  Switch smart_mode_sw = 4;
//	}
//	Parm.Room.Custom { CleanType clean_type = 1; Fan fan = 2; MopMode mop_mode = 3;
//	                   CleanExtent clean_extent = 4; uint32 clean_times = 5 }
//
// The robot persists all of it in the map's rooms_param.yaml, field for field,
// which is where reads come from - no round trip, and it is the copy that
// survives a reboot.
//
// custom_enable is worth watching. It ships 0, meaning the per-room parameters
// are stored but ignored and every room gets the global ones. Setting per-room
// values without turning it on looks like it worked and changes nothing.

import (
	"bufio"
	"context"
	"fmt"
	"github.com/dhumpf/x10-ctl/internal/pb"
	"net/http"
	"os"
	"path/filepath"
	"sort"
	"strconv"
	"strings"
	"time"
)

const (
	mdsMapEditRequest  = "eros.proto.msg.app.MapEditRequest"
	mdsMapEditResponse = "eros.proto.msg.app.MapEditResponse"
)

// RoomParams is one room's cleaning parameters.
//
// mop_water is MopMode.level; the YAML calls it mop_water and the proto calls
// it level, and they are the same 0..2.
type RoomParams struct {
	CleanType   int `json:"clean_type"`
	Fan         int `json:"fan"`
	MopWater    int `json:"mop_water"`
	CornerClean int `json:"corner_clean"`
	CleanExtent int `json:"clean_extent"`
	CleanTimes  int `json:"clean_times"`
}

// RoomDetail is a room with its name, order and parameters.
type RoomDetail struct {
	ID     int         `json:"id"`
	Name   string      `json:"name"`
	Order  int         `json:"order"`
	Params *RoomParams `json:"params,omitempty"`
}

type RoomsView struct {
	MapID        int          `json:"map_id"`
	MapName      string       `json:"map_name"`
	CustomEnable bool         `json:"custom_enable"`
	SmartMode    bool         `json:"smart_mode"`
	Rooms        []RoomDetail `json:"rooms"`
}

func atoiOr(s string, def int) int {
	if v, err := strconv.Atoi(strings.TrimSpace(s)); err == nil {
		return v
	}
	return def
}

// readRoomsNested parses rooms_param.yaml by indentation depth.
//
// readRoomsParam in rooms.go is two-level and cannot do this one: rooms_custom
// nests a third level, so its per-room blocks collapse into each other and
// every room reads back as the last one parsed. Depth is taken from the leading
// whitespace, which the robot writes consistently.
func readRoomsNested(path string) (map[string]map[string]map[string]string, error) {
	f, err := os.Open(path)
	if err != nil {
		return nil, err
	}
	defer f.Close()

	out := map[string]map[string]map[string]string{}
	section, sub := "", ""
	sc := bufio.NewScanner(f)
	for sc.Scan() {
		line := sc.Text()
		trimmed := strings.TrimSpace(line)
		if trimmed == "" || strings.HasPrefix(trimmed, "%YAML") || trimmed == "---" {
			continue
		}
		key, val, found := strings.Cut(trimmed, ":")
		if !found {
			continue
		}
		key, val = strings.TrimSpace(key), strings.TrimSpace(val)
		depth := len(line) - len(strings.TrimLeft(line, " \t"))

		switch {
		case depth == 0:
			section, sub = key, ""
			if out[section] == nil {
				out[section] = map[string]map[string]string{}
			}
			if val != "" && val != "{}" {
				out[section][""] = map[string]string{"": val}
			}
		case section == "":
		case val == "" || val == "{}":
			// A key with no value opens a third level, e.g. rooms_custom.room1.
			sub = key
			if out[section][sub] == nil {
				out[section][sub] = map[string]string{}
			}
		case sub == "":
			if out[section][""] == nil {
				out[section][""] = map[string]string{}
			}
			out[section][""][key] = val
		default:
			out[section][sub][key] = val
		}
	}
	return out, sc.Err()
}

// readRoomDetails parses the whole of rooms_param.yaml.
func readRoomDetails() (*RoomsView, error) {
	meta, err := readMapMeta()
	if err != nil {
		return nil, err
	}
	path := filepath.Join(meta.Dir, "rooms_param.yaml")
	nested, err := readRoomsNested(path)
	if err != nil {
		return nil, err
	}
	flat := func(name string) map[string]string {
		if m, ok := nested[name][""]; ok {
			return m
		}
		return map[string]string{}
	}
	scalar := func(name string) string { return flat(name)[""] }

	view := &RoomsView{MapID: meta.ID}
	if n, ok := scanKey(filepath.Join(meta.Dir, "map_name.yaml"), "map_name"); ok {
		view.MapName = strings.Trim(n, `"`)
	}
	view.CustomEnable = strings.TrimSpace(scalar("custom_enable")) == "1"
	view.SmartMode = strings.TrimSpace(scalar("smart_mode_sw")) == "1"

	names, order := flat("rooms_name"), flat("rooms_order")
	for key, name := range names {
		if !strings.HasPrefix(key, "room") {
			continue
		}
		id, err := strconv.Atoi(strings.TrimPrefix(key, "room"))
		if err != nil {
			continue
		}
		d := RoomDetail{ID: id, Name: strings.Trim(name, `"`)}
		if v, ok := order[key]; ok {
			d.Order = atoiOr(v, 0)
		}
		if p, ok := nested["rooms_custom"][key]; ok {
			d.Params = &RoomParams{
				CleanType:   atoiOr(p["clean_type"], 0),
				Fan:         atoiOr(p["fan"], 0),
				MopWater:    atoiOr(p["mop_water"], 0),
				CornerClean: atoiOr(p["corner_clean"], 0),
				CleanExtent: atoiOr(p["clean_extent"], 0),
				CleanTimes:  atoiOr(p["clean_times"], 1),
			}
		}
		view.Rooms = append(view.Rooms, d)
	}
	sort.Slice(view.Rooms, func(i, j int) bool { return view.Rooms[i].ID < view.Rooms[j].ID })
	if len(view.Rooms) == 0 {
		return nil, fmt.Errorf("no rooms in %s", path)
	}
	return view, nil
}

// roomsCustom encodes a RoomsCustom.
//
// enable is a pointer so "leave the master switch alone" is expressible: it is
// the difference between changing a room's suction and silently turning the
// whole per-room feature on.
func roomsCustom(rooms []RoomDetail, enable *bool, smart *bool) []byte {
	var rc []byte

	if enable != nil {
		var inner []byte
		if *enable {
			inner = pb.FieldVarint(1, 1)
		}
		rc = append(rc, pb.FieldBytes(1, inner)...)
	}

	var orderBody []byte
	var parmBody []byte
	for _, r := range rooms {
		if r.Order > 0 {
			room := append(pb.FieldVarint(1, uint64(r.ID)), pb.FieldVarint(2, uint64(r.Order))...)
			orderBody = append(orderBody, pb.FieldBytes(1, room)...)
		}
		if r.Params == nil {
			continue
		}
		p := r.Params
		var custom []byte
		enumField := func(field uint64, v int) {
			var inner []byte
			if v != 0 {
				inner = pb.FieldVarint(1, uint64(v))
			}
			custom = append(custom, pb.FieldBytes(field, inner)...)
		}
		enumField(1, p.CleanType)
		enumField(2, p.Fan)
		// MopMode carries level and corner_clean together, so it is built by
		// hand rather than through enumField.
		var mop []byte
		if p.MopWater != 0 {
			mop = pb.FieldVarint(1, uint64(p.MopWater))
		}
		if p.CornerClean != 0 {
			mop = append(mop, pb.FieldVarint(2, uint64(p.CornerClean))...)
		}
		custom = append(custom, pb.FieldBytes(3, mop)...)
		enumField(4, p.CleanExtent)
		if p.CleanTimes > 0 {
			custom = append(custom, pb.FieldVarint(5, uint64(p.CleanTimes))...)
		}

		room := append(pb.FieldVarint(1, uint64(r.ID)), pb.FieldBytes(2, custom)...)
		parmBody = append(parmBody, pb.FieldBytes(1, room)...)
	}
	if len(orderBody) > 0 {
		rc = append(rc, pb.FieldBytes(2, orderBody)...)
	}
	if len(parmBody) > 0 {
		rc = append(rc, pb.FieldBytes(3, parmBody)...)
	}
	if smart != nil {
		var inner []byte
		if *smart {
			inner = pb.FieldVarint(1, 1)
		}
		rc = append(rc, pb.FieldBytes(4, inner)...)
	}
	return rc
}

// Map edit methods, from MapEditRequest.Method.
const (
	mapEditMergeRooms  uint64 = 0
	mapEditDivideRoom  uint64 = 1
	mapEditRenameRoom  uint64 = 3
	mapEditRestricted  uint64 = 4
	mapEditRoomsCustom uint64 = 5
	mapEditRevert      uint64 = 9
	mapEditBatchRename uint64 = 10
	mapEditDrawZones   uint64 = 12
)

// mapEdit publishes one MapEditRequest and waits for the robot's verdict.
//
// The verdict matters more here than on other channels: mds_pub reports success
// for anything it transmits, so a rejected map edit is otherwise silent. The
// first version of this omitted method, seq and map_id entirely and every edit
// failed while reporting success.
func (l *Local) mapEdit(ctx context.Context, method, seq uint64, mapID int, op []byte) error {
	req := append(pb.FieldVarint(1, method), pb.FieldVarint(2, seq)...)
	if mapID > 0 {
		req = append(req, pb.FieldVarint(3, uint64(mapID))...)
	}
	req = append(req, op...)
	if err := l.mdsPublish(ctx, mdsMapEditRequest, pb.FieldBytes(2, req)); err != nil {
		return err
	}
	return l.awaitMapEdit(ctx, seq)
}

// awaitMapEdit reads MapEditResponse until it sees our sequence.
//
//	MapEditResponse { Method method = 1; uint32 seq = 2;
//	                  Result result = 3;   // SUCCESS 0, STARTED 1, FAILED 2
//	                  FailCode fail_code = 4 }
func (l *Local) awaitMapEdit(ctx context.Context, seq uint64) error {
	failCodes := map[uint64]string{
		0: "unknown", 1: "area too small", 2: "rooms not adjacent",
		3: "too many rooms", 4: "operation is locked", 5: "area too large",
	}
	deadline := time.Now().Add(4 * time.Second)
	for time.Now().Before(deadline) {
		if rec, err := mdsNewest(mdsMapEditResponse); err == nil {
			var gotSeq, result, fail uint64
			var haveSeq bool
			for _, f := range pb.ParseFields(rec) {
				if f.Num != 2 || f.Wire != 2 {
					continue
				}
				for _, g := range pb.ParseFields(f.Data) {
					switch {
					case g.Num == 2 && g.Wire == 0:
						gotSeq, haveSeq = g.Val, true
					case g.Num == 3 && g.Wire == 0:
						result = g.Val
					case g.Num == 4 && g.Wire == 2:
						for _, h := range pb.ParseFields(g.Data) {
							if h.Num == 1 && h.Wire == 0 {
								fail = h.Val
							}
						}
					}
				}
			}
			if haveSeq && gotSeq == seq {
				if result == 2 {
					return fmt.Errorf("robot rejected the map edit: %s", failCodes[fail])
				}
				return nil
			}
		}
		select {
		case <-ctx.Done():
			return ctx.Err()
		case <-time.After(150 * time.Millisecond):
		}
	}
	return fmt.Errorf("no MapEditResponse for seq %d", seq)
}

// SetRoomsCustom publishes a RoomsCustom map edit.
func (l *Local) SetRoomsCustom(ctx context.Context, seq uint64, mapID int, rooms []RoomDetail, enable, smart *bool) error {
	rc := roomsCustom(rooms, enable, smart)
	if len(rc) == 0 {
		return fmt.Errorf("nothing to set")
	}
	return l.mapEdit(ctx, mapEditRoomsCustom, seq, mapID, pb.FieldBytes(10, rc))
}

func (s *server) handleRoomSettings(w http.ResponseWriter, r *http.Request) {
	if r.Method == http.MethodGet {
		view, err := readRoomDetails()
		if err != nil {
			writeJSON(w, http.StatusServiceUnavailable, map[string]any{"ok": false, "error": err.Error()})
			return
		}
		writeJSON(w, http.StatusOK, map[string]any{
			"ok": true, "view": view,
			"names": map[string]any{
				"clean_type": typeNames, "fan": suctionNames,
				"mop_water": mopNames, "clean_extent": extentNames,
			},
		})
		return
	}
	if r.Method != http.MethodPost {
		writeJSON(w, http.StatusMethodNotAllowed, map[string]string{"error": "GET or POST"})
		return
	}

	var body struct {
		Rooms        []RoomDetail `json:"rooms"`
		CustomEnable *bool        `json:"custom_enable"`
		SmartMode    *bool        `json:"smart_mode"`
	}
	if err := decodeJSON(w, r, &body); err != nil {
		writeJSON(w, http.StatusBadRequest, map[string]any{"ok": false, "error": "bad json"})
		return
	}
	for _, rm := range body.Rooms {
		if rm.Params == nil {
			continue
		}
		for _, c := range []struct {
			v    int
			list []string
			name string
		}{
			{rm.Params.CleanType, typeNames, "clean_type"},
			{rm.Params.Fan, suctionNames, "fan"},
			{rm.Params.MopWater, mopNames, "mop_water"},
			{rm.Params.CleanExtent, extentNames, "clean_extent"},
		} {
			if c.v < 0 || c.v >= len(c.list) {
				writeJSON(w, http.StatusBadRequest, map[string]any{
					"ok": false, "error": fmt.Sprintf("room %d: %s out of range", rm.ID, c.name),
				})
				return
			}
		}
	}

	ctx, cancel := context.WithTimeout(r.Context(), 15*time.Second)
	defer cancel()
	mapID := 0
	if v, err := readMapMeta(); err == nil {
		mapID = v.ID
	}
	if err := s.local.SetRoomsCustom(ctx, s.nextSeq(), mapID, body.Rooms, body.CustomEnable, body.SmartMode); err != nil {
		writeJSON(w, http.StatusOK, map[string]any{"ok": false, "error": err.Error()})
		return
	}

	// The robot rewrites rooms_param.yaml while handling this.
	select {
	case <-ctx.Done():
	case <-time.After(900 * time.Millisecond):
	}
	view, _ := readRoomDetails()
	writeJSON(w, http.StatusOK, map[string]any{"ok": true, "bus": "local", "view": view})
}

// RenameRoom sets a room's name.
//
//	MapEditRequest { RoomDesc room_desc = 9 }
//	RoomDesc       { uint32 id = 1; string name = 2; Floor floor = 3; RoomScene scene = 4 }
//
// floor and scene are left out: they are separate properties with their own
// enums, and an edit that only means to change the name should not carry
// guesses about the rest.
func (l *Local) RenameRoom(ctx context.Context, seq uint64, mapID, id int, name string) error {
	if name == "" {
		return fmt.Errorf("name is required")
	}
	desc := append(pb.FieldVarint(1, uint64(id)), pb.FieldBytes(2, []byte(name))...)
	return l.mapEdit(ctx, mapEditRenameRoom, seq, mapID, pb.FieldBytes(9, desc))
}

func (s *server) handleRoomRename(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPost {
		writeJSON(w, http.StatusMethodNotAllowed, map[string]string{"error": "POST only"})
		return
	}
	var body struct {
		ID   int    `json:"id"`
		Name string `json:"name"`
	}
	if err := decodeJSON(w, r, &body); err != nil {
		writeJSON(w, http.StatusBadRequest, map[string]any{"ok": false, "error": "bad json"})
		return
	}
	body.Name = strings.TrimSpace(body.Name)
	if body.ID <= 0 || body.Name == "" {
		writeJSON(w, http.StatusBadRequest, map[string]any{"ok": false, "error": "id and name are required"})
		return
	}
	// RENAME_ROOM accepts an id that does not exist and creates a phantom entry
	// in rooms_name that nothing can remove afterwards: BATCH_RENAME_ROOM adds
	// and updates but never shortens the list, and there is no delete method.
	// Found by testing a bad id, which left a room 99 behind. Guard here rather
	// than trust the robot to reject it.
	known, err := readRoomDetails()
	if err != nil {
		writeJSON(w, http.StatusServiceUnavailable, map[string]any{"ok": false, "error": err.Error()})
		return
	}
	found := false
	for _, r := range known.Rooms {
		if r.ID == body.ID {
			found = true
			break
		}
	}
	if !found {
		writeJSON(w, http.StatusBadRequest, map[string]any{
			"ok": false, "error": fmt.Sprintf("no room %d on this map", body.ID),
		})
		return
	}
	if len(body.Name) > 48 {
		writeJSON(w, http.StatusBadRequest, map[string]any{"ok": false, "error": "name too long"})
		return
	}

	ctx, cancel := context.WithTimeout(r.Context(), 15*time.Second)
	defer cancel()
	mapID := 0
	if v, err := readMapMeta(); err == nil {
		mapID = v.ID
	}
	if err := s.local.RenameRoom(ctx, s.nextSeq(), mapID, body.ID, body.Name); err != nil {
		writeJSON(w, http.StatusOK, map[string]any{"ok": false, "error": err.Error()})
		return
	}
	select {
	case <-ctx.Done():
	case <-time.After(900 * time.Millisecond):
	}
	view, _ := readRoomDetails()
	writeJSON(w, http.StatusOK, map[string]any{"ok": true, "bus": "local", "view": view})
}

// BatchRename replaces the room name list in one edit.
//
// Its use here is repair rather than convenience: RENAME_ROOM accepts an id
// that does not exist and creates a phantom entry, so a bad id leaves rubbish
// in rooms_name that no single-room edit can remove. Sending the full set is
// the only lever that can shorten the list.
//
//	MapEditRequest { BatchRoomDesc batch_room_desc = 15 }
//	BatchRoomDesc  { repeated RoomDesc rooms = 1 }
func (l *Local) BatchRename(ctx context.Context, seq uint64, mapID int, rooms []RoomDetail) error {
	var body []byte
	for _, r := range rooms {
		if r.ID <= 0 || r.Name == "" {
			continue
		}
		desc := append(pb.FieldVarint(1, uint64(r.ID)), pb.FieldBytes(2, []byte(r.Name))...)
		body = append(body, pb.FieldBytes(1, desc)...)
	}
	if len(body) == 0 {
		return fmt.Errorf("no rooms given")
	}
	return l.mapEdit(ctx, mapEditBatchRename, seq, mapID, pb.FieldBytes(15, body))
}

func (s *server) handleRoomsBatch(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPost {
		writeJSON(w, http.StatusMethodNotAllowed, map[string]string{"error": "POST only"})
		return
	}
	var body struct {
		Rooms []RoomDetail `json:"rooms"`
	}
	if err := decodeJSON(w, r, &body); err != nil {
		writeJSON(w, http.StatusBadRequest, map[string]any{"ok": false, "error": "bad json"})
		return
	}
	ctx, cancel := context.WithTimeout(r.Context(), 15*time.Second)
	defer cancel()
	mapID := 0
	if v, err := readMapMeta(); err == nil {
		mapID = v.ID
	}
	if err := s.local.BatchRename(ctx, s.nextSeq(), mapID, body.Rooms); err != nil {
		writeJSON(w, http.StatusOK, map[string]any{"ok": false, "error": err.Error()})
		return
	}
	select {
	case <-ctx.Done():
	case <-time.After(900 * time.Millisecond):
	}
	view, _ := readRoomDetails()
	writeJSON(w, http.StatusOK, map[string]any{"ok": true, "bus": "local", "view": view})
}

// MergeRooms merges two or more adjacent rooms into one.
//
//	MapEditRequest { MergeRooms merge_rooms = 6 }
//	MergeRooms     { repeated uint32 room_ids = 1 }
//
// This is how the room count comes down - there is no delete. The robot checks
// adjacency itself and answers ROOM_UNADJACENT if the rooms do not touch, which
// awaitMapEdit surfaces rather than swallowing, so a refused merge is not
// mistaken for a done one.
//
// Which room survives is the robot's choice, not ours: the request carries only
// the ids. Read the names back afterwards rather than assuming the first one
// won.
func (l *Local) MergeRooms(ctx context.Context, seq uint64, mapID int, ids []int) error {
	if len(ids) < 2 {
		return fmt.Errorf("merging needs at least two rooms")
	}
	var body []byte
	for _, id := range ids {
		if id <= 0 {
			return fmt.Errorf("bad room id %d", id)
		}
		body = append(body, pb.FieldVarint(1, uint64(id))...)
	}
	return l.mapEdit(ctx, mapEditMergeRooms, seq, mapID, pb.FieldBytes(6, body))
}

func (s *server) handleRoomsMerge(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPost {
		writeJSON(w, http.StatusMethodNotAllowed, map[string]string{"error": "POST only"})
		return
	}
	var body struct {
		Rooms []int `json:"rooms"`
	}
	if err := decodeJSON(w, r, &body); err != nil {
		writeJSON(w, http.StatusBadRequest, map[string]any{"ok": false, "error": "bad json"})
		return
	}
	if len(body.Rooms) < 2 {
		writeJSON(w, http.StatusBadRequest, map[string]any{
			"ok": false, "error": "pick at least two rooms",
		})
		return
	}
	known, err := readRoomDetails()
	if err != nil {
		writeJSON(w, http.StatusServiceUnavailable, map[string]any{"ok": false, "error": err.Error()})
		return
	}
	have := map[int]bool{}
	for _, r := range known.Rooms {
		have[r.ID] = true
	}
	for _, id := range body.Rooms {
		if !have[id] {
			writeJSON(w, http.StatusBadRequest, map[string]any{
				"ok": false, "error": fmt.Sprintf("no room %d on this map", id),
			})
			return
		}
	}

	ctx, cancel := context.WithTimeout(r.Context(), 20*time.Second)
	defer cancel()
	if err := s.local.MergeRooms(ctx, s.nextSeq(), known.MapID, body.Rooms); err != nil {
		writeJSON(w, http.StatusOK, map[string]any{"ok": false, "error": err.Error()})
		return
	}
	// Merging rewrites the room map, not just a name, so give it longer.
	select {
	case <-ctx.Done():
	case <-time.After(2 * time.Second):
	}
	view, _ := readRoomDetails()
	writeJSON(w, http.StatusOK, map[string]any{"ok": true, "bus": "local", "view": view})
}

// Map edit revert types, from MapEditRequest.RevertEdit.Type.
const (
	revertLast uint64 = 0 // undo the last edit
	revertDone uint64 = 1 // finish the edit session
)

// RevertEdit undoes the last map edit, or ends the editing session.
//
//	MapEditRequest { RevertEdit revert_edit = 14 }
//	RevertEdit     { Type type = 1 }   // LAST 0, DONE 1
//
// The existence of DONE says map edits are staged rather than applied one by
// one. Rename and merge both take effect without it, so whatever DONE closes is
// not needed for a single change - but it is worth sending after a batch, and
// it is the reason an undo exists at all.
//
// LAST is one step. There is no depth in the message, so a mistake several
// edits back needs several calls, and nothing reports how many are available.
func (l *Local) RevertEdit(ctx context.Context, seq uint64, mapID int, kind uint64) error {
	var body []byte
	if kind != 0 {
		body = pb.FieldVarint(1, kind)
	}
	return l.mapEdit(ctx, mapEditRevert, seq, mapID, pb.FieldBytes(14, body))
}

func (s *server) handleMapRevert(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPost {
		writeJSON(w, http.StatusMethodNotAllowed, map[string]string{"error": "POST only"})
		return
	}
	kind := revertLast
	action := strings.ToLower(strings.TrimSpace(r.URL.Query().Get("action")))
	switch action {
	case "", "undo", "last":
		kind = revertLast
	case "done", "finish":
		kind = revertDone
	default:
		writeJSON(w, http.StatusBadRequest, map[string]any{
			"ok": false, "error": "action must be undo or done",
		})
		return
	}

	ctx, cancel := context.WithTimeout(r.Context(), 20*time.Second)
	defer cancel()
	mapID := 0
	if v, err := readMapMeta(); err == nil {
		mapID = v.ID
	}
	if err := s.local.RevertEdit(ctx, s.nextSeq(), mapID, kind); err != nil {
		writeJSON(w, http.StatusOK, map[string]any{"ok": false, "error": err.Error()})
		return
	}
	select {
	case <-ctx.Done():
	case <-time.After(2 * time.Second):
	}
	view, _ := readRoomDetails()
	writeJSON(w, http.StatusOK, map[string]any{"ok": true, "action": action, "bus": "local", "view": view})
}

// DivideRoom splits a room along a line.
//
//	MapEditRequest { DivideRoom divide_room = 7 }   // method 1
//	DivideRoom     { uint32 room_id = 1; repeated Point points = 2 }
//	Point          { sint32 x = 1; sint32 y = 2 }   // zigzag, centimetres
//
// Units and encoding are the same as the keep-out zones, which were measured
// rather than guessed - see zones.go. Two points make the cut.
//
// The robot decides whether the split is legal and answers AREA_SMALL when a
// side would be too small, which awaitMapEdit surfaces. Which half keeps the
// original id and name is the robot's call, so read the rooms back afterwards.
func (l *Local) DivideRoom(ctx context.Context, seq uint64, mapID, roomID int, pts []ZonePoint) error {
	if len(pts) != 2 {
		return fmt.Errorf("a split needs exactly two points, got %d", len(pts))
	}
	body := pb.FieldVarint(1, uint64(roomID))
	for _, p := range pts {
		body = append(body, cmPoint(2, p)...)
	}
	return l.mapEdit(ctx, mapEditDivideRoom, seq, mapID, pb.FieldBytes(7, body))
}

func (s *server) handleRoomDivide(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPost {
		writeJSON(w, http.StatusMethodNotAllowed, map[string]string{"error": "POST only"})
		return
	}
	var body struct {
		Room   int         `json:"room"`
		Points []ZonePoint `json:"points"`
	}
	if err := decodeJSON(w, r, &body); err != nil {
		writeJSON(w, http.StatusBadRequest, map[string]any{"ok": false, "error": "bad json"})
		return
	}
	if len(body.Points) != 2 {
		writeJSON(w, http.StatusBadRequest, map[string]any{
			"ok": false, "error": "draw a line across the room - two points",
		})
		return
	}
	known, err := readRoomDetails()
	if err != nil {
		writeJSON(w, http.StatusServiceUnavailable, map[string]any{"ok": false, "error": err.Error()})
		return
	}
	found := false
	for _, rm := range known.Rooms {
		if rm.ID == body.Room {
			found = true
		}
	}
	if !found {
		writeJSON(w, http.StatusBadRequest, map[string]any{
			"ok": false, "error": fmt.Sprintf("no room %d on this map", body.Room),
		})
		return
	}

	ctx, cancel := context.WithTimeout(r.Context(), 25*time.Second)
	defer cancel()
	if err := s.local.DivideRoom(ctx, s.nextSeq(), known.MapID, body.Room, body.Points); err != nil {
		writeJSON(w, http.StatusOK, map[string]any{"ok": false, "error": err.Error()})
		return
	}
	select {
	case <-ctx.Done():
	case <-time.After(2 * time.Second):
	}
	view, _ := readRoomDetails()
	writeJSON(w, http.StatusOK, map[string]any{"ok": true, "bus": "local", "view": view})
}
