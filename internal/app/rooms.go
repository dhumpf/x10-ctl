package app

// Rooms, and cleaning a named subset of them.
//
// The names and per-room settings live in the map directory as
// rooms_param.yaml, so no cloud round trip is needed to know that room 2 is
// "Living Room". DP 165 carries the same list over MQTT as an opaque blob.

import (
	"bufio"
	"encoding/json"
	"github.com/dhumpf/x10-ctl/internal/cloud"
	"github.com/dhumpf/x10-ctl/internal/dp"
	"github.com/dhumpf/x10-ctl/internal/pb"
	"log"
	"net/http"
	"os"
	"path/filepath"
	"sort"
	"strconv"
	"strings"
)

type Room struct {
	ID    int    `json:"id"`
	Name  string `json:"name"`
	Index int    `json:"index"`
}

// readRoomsParam parses the nested sections of rooms_param.yaml.
//
// scanKey cannot do this one: `rooms_index` and `rooms_name` both contain keys
// called room1..room8, so a flat first-match scan returns the wrong section's
// value. Section-aware, indentation-driven, and small enough not to want a YAML
// dependency for it.
func readRoomsParam(path string) (map[string]map[string]string, error) {
	f, err := os.Open(path)
	if err != nil {
		return nil, err
	}
	defer f.Close()

	out := map[string]map[string]string{}
	section := ""
	sc := bufio.NewScanner(f)
	for sc.Scan() {
		line := sc.Text()
		trimmed := strings.TrimSpace(line)
		if trimmed == "" || strings.HasPrefix(trimmed, "%YAML") || trimmed == "---" {
			continue
		}
		indented := line != trimmed
		key, val, found := strings.Cut(trimmed, ":")
		if !found {
			continue
		}
		key, val = strings.TrimSpace(key), strings.TrimSpace(val)
		if !indented {
			section = key
			if out[section] == nil {
				out[section] = map[string]string{}
			}
			if val != "" && val != "{}" {
				out[section][""] = val
			}
			continue
		}
		if section == "" {
			continue
		}
		out[section][key] = val
	}
	return out, sc.Err()
}

func readRooms() ([]Room, string, error) {
	meta, err := readMapMeta()
	if err != nil {
		return nil, "", err
	}
	sections, err := readRoomsParam(filepath.Join(meta.Dir, "rooms_param.yaml"))
	if err != nil {
		return nil, "", err
	}
	mapName := ""
	if n, ok := scanKey(filepath.Join(meta.Dir, "map_name.yaml"), "map_name"); ok {
		mapName = strings.Trim(n, `"`)
	}

	names, idx := sections["rooms_name"], sections["rooms_index"]
	var rooms []Room
	for k, name := range names {
		if !strings.HasPrefix(k, "room") {
			continue
		}
		id, err := strconv.Atoi(strings.TrimPrefix(k, "room"))
		if err != nil {
			continue
		}
		r := Room{ID: id, Name: strings.Trim(name, `"`)}
		if v, ok := idx[k]; ok {
			r.Index, _ = strconv.Atoi(v)
		}
		rooms = append(rooms, r)
	}
	sort.Slice(rooms, func(i, j int) bool { return rooms[i].ID < rooms[j].ID })
	return rooms, mapName, nil
}

func (s *server) handleRooms(w http.ResponseWriter, r *http.Request) {
	rooms, mapName, err := readRooms()
	if err != nil {
		writeJSON(w, http.StatusServiceUnavailable, map[string]any{"ok": false, "error": err.Error()})
		return
	}
	meta, _ := readMapMeta()
	id := 0
	if meta != nil {
		id = meta.ID
	}
	writeJSON(w, http.StatusOK, map[string]any{
		"ok": true, "map_id": id, "map_name": mapName, "rooms": rooms,
	})
}

// roomsParam encodes SelectRoomsClean, field 4 of ModeCtrlRequest.
//
//	repeated Room { id = 1, order = 2 }  in field 1
//	clean_times = 2
//	map_id      = 3
//
// Order is the cleaning order, one-based, in the order given. The encoding
// matches the frames the app sends.
func roomsParam(rooms []int, mapID, cleanTimes uint64) []byte {
	var body []byte
	for i, id := range rooms {
		room := append(pb.FieldVarint(1, uint64(id)), pb.FieldVarint(2, uint64(i+1))...)
		body = append(body, pb.FieldBytes(1, room)...)
	}
	body = append(body, pb.FieldVarint(2, cleanTimes)...)
	return append(body, pb.FieldVarint(3, mapID)...)
}

// handleRoomClean starts a room-selective clean over whichever channel is up.
func (s *server) handleRoomClean(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPost {
		writeJSON(w, http.StatusMethodNotAllowed, map[string]string{"error": "POST only"})
		return
	}
	var body struct {
		Rooms      []int  `json:"rooms"`
		MapID      uint64 `json:"map_id"`
		CleanTimes uint64 `json:"clean_times"`
	}
	if err := decodeJSON(w, r, &body); err != nil {
		writeJSON(w, http.StatusBadRequest, map[string]any{"ok": false, "error": "bad json"})
		return
	}
	if len(body.Rooms) == 0 {
		writeJSON(w, http.StatusBadRequest, map[string]any{"ok": false, "error": "no rooms given"})
		return
	}
	if body.MapID == 0 {
		meta, err := readMapMeta()
		if err != nil {
			writeJSON(w, http.StatusServiceUnavailable, map[string]any{"ok": false, "error": err.Error()})
			return
		}
		body.MapID = uint64(meta.ID)
	}
	if body.CleanTimes == 0 {
		body.CleanTimes = 1
	}

	// Parameters first, then the start. Without this the room's stored settings
	// win, and AI automation stores sweep_and_mop - see docs/app-control.md.
	if set, err := readSettings(); err == nil {
		rp := make([]cloud.RoomParam, 0, len(body.Rooms))
		for _, id := range body.Rooms {
			p := cloud.RoomParam{ID: id, CleanTimes: int(body.CleanTimes)}
			if set.CleanType != nil {
				p.CleanType = *set.CleanType
			}
			if set.Fan != nil {
				p.Suction = *set.Fan
			}
			if set.MopWater != nil {
				p.Water = *set.MopWater
			}
			if set.Extent != nil {
				p.Intensity = *set.Extent
			}
			rp = append(rp, p)
		}
		if err := s.cloud.SendRoomParams(body.MapID, rp); err != nil {
			log.Printf("roomclean: per-room params not sent (%v); the room's own settings will win", err)
		}
	}

	param := roomsParam(body.Rooms, body.MapID, body.CleanTimes)
	via, err := s.modeCtrlParam(r, dp.MethodRoomClean, param)
	if err != nil {
		writeJSON(w, http.StatusOK, map[string]any{"ok": false, "error": err.Error()})
		return
	}
	writeJSON(w, http.StatusOK, map[string]any{
		"ok": true, "command": "roomclean", "bus": via,
		"rooms": body.Rooms, "map_id": body.MapID, "clean_times": body.CleanTimes,
	})
}

func decodeJSON(w http.ResponseWriter, r *http.Request, v any) error {
	return json.NewDecoder(http.MaxBytesReader(w, r.Body, 8192)).Decode(v)
}
