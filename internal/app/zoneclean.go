package app

// Zone cleaning: clean a rectangle rather than a room.
//
// WORLD centimetres, zigzag encoded, no mirror anywhere. Quadrangle carries the
// same Point as the keep-out zones - `sint32 x = 1` - so it needs fieldSint.
//
// The both-axes mirror this file used to claim was an artefact of reading that
// zigzag as a plain varint. Decoded properly, the 2026-08-07 app capture reads
// x -361..-570, y -76..+133, which lands inside the Living Room's own world
// extent of x -636..-132, y -212..+192. Nothing was ever flipped.
//
// Confirmed against the dock, the one point readable in two frames at once:
// layer (921, 1350) + origin (-947.5, -1352.5) cm = (-26.5, -2.5), and
// /api/pose reports the dock at (-26.7, -2.9) cm.
//
// Sent as plain varints the robot decoded every coordinate as a large negative
// number and answered "cannot reach target" - five runs on 2026-08-10, in four
// different candidate frames, all failing for this one reason.

import (
	"context"
	"fmt"
	"github.com/dhumpf/x10-ctl/internal/dp"
	"github.com/dhumpf/x10-ctl/internal/pb"
	"math"
	"net/http"
	"time"
)

// ZoneRect is one rectangle to clean, in map-layer centimetres - the space
// /api/map.png draws in. zonesParam shifts it to world by the map origin.
type ZoneRect struct {
	X1 int `json:"x1"`
	Y1 int `json:"y1"`
	X2 int `json:"x2"`
	Y2 int `json:"y2"`
}

// zonesParam builds the SelectZonesClean body, untagged: ModeCtrlParam wraps it
// in field 5. Layer centimetres in, world centimetres out, zigzag encoded.
// Corners go round the ring.
//
// originX and originY are the map origin in centimetres, negative on this
// robot. They are what turns a layer coordinate into the world one the robot
// navigates in, and dropping them is the same class of bug that put keep-out
// zones most of a map away.
func zonesParam(rects []ZoneRect, mapID, cleanTimes uint64, originX, originY int) []byte {
	var zones []byte
	for _, r := range rects {
		corner := func(field uint64, x, y int) []byte {
			var p []byte
			p = append(p, pb.FieldSint(1, int64(x+originX))...)
			p = append(p, pb.FieldSint(2, int64(y+originY))...)
			return pb.FieldBytes(field, p)
		}
		var poly []byte
		poly = append(poly, corner(1, r.X1, r.Y1)...)
		poly = append(poly, corner(2, r.X2, r.Y1)...)
		poly = append(poly, corner(3, r.X2, r.Y2)...)
		poly = append(poly, corner(4, r.X1, r.Y2)...)

		var zone []byte
		zone = append(zone, pb.FieldBytes(1, poly)...)
		zone = append(zone, pb.FieldVarint(2, cleanTimes)...)
		zones = append(zones, pb.FieldBytes(1, zone)...)
	}
	return append(zones, pb.FieldVarint(2, mapID)...)
}

// originCM is the map origin in whole centimetres. It refuses rather than
// guesses: a zero origin silently sends coordinates a whole map away, which is
// the failure this file already had once.
func originCM(meta *mapMeta) (int, int, error) {
	if meta == nil || len(meta.Origin) < 2 {
		return 0, 0, fmt.Errorf("map has no origin")
	}
	return int(math.Round(meta.Origin[0] * 100)), int(math.Round(meta.Origin[1] * 100)), nil
}

func (s *server) handleZoneClean(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPost {
		writeJSON(w, http.StatusMethodNotAllowed, map[string]string{"error": "POST only"})
		return
	}
	var body struct {
		Zones      []ZoneRect `json:"zones"`
		MapID      uint64     `json:"map_id"`
		CleanTimes uint64     `json:"clean_times"`
	}
	if err := decodeJSON(w, r, &body); err != nil {
		writeJSON(w, http.StatusBadRequest, map[string]any{"ok": false, "error": "bad json"})
		return
	}
	if len(body.Zones) == 0 {
		writeJSON(w, http.StatusBadRequest, map[string]any{
			"ok": false, "error": `want {"zones":[{"x1":721,"y1":151,"x2":1139,"y2":266}]} in centimetres`,
		})
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

	ctx, cancel := context.WithTimeout(r.Context(), 15*time.Second)
	defer cancel()
	meta, err := readMapMeta()
	if err != nil {
		writeJSON(w, http.StatusServiceUnavailable, map[string]any{"ok": false, "error": err.Error()})
		return
	}
	ox, oy, err := originCM(meta)
	if err != nil {
		writeJSON(w, http.StatusServiceUnavailable, map[string]any{"ok": false, "error": err.Error()})
		return
	}
	param := zonesParam(body.Zones, body.MapID, body.CleanTimes, ox, oy)
	via, err := s.modeCtrlParamCtx(ctx, dp.MethodZonesClean, param)
	if err != nil {
		writeJSON(w, http.StatusOK, map[string]any{"ok": false, "error": err.Error()})
		return
	}
	writeJSON(w, http.StatusOK, map[string]any{
		"ok": true, "command": "zoneclean", "bus": via,
		"zones": body.Zones, "map_id": body.MapID, "clean_times": body.CleanTimes,
	})
}

// spotParam is the SpotClean body. There is no coordinate in it: the robot
// cleans where it stands, which is why the app makes you drive there first.
// Untagged, like every other param encoder - ModeCtrlParam adds field 6.
func spotParam() []byte {
	return pb.FieldVarint(1, 1)
}

func (s *server) handleSpotClean(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPost {
		writeJSON(w, http.StatusMethodNotAllowed, map[string]string{"error": "POST only"})
		return
	}
	ctx, cancel := context.WithTimeout(r.Context(), 15*time.Second)
	defer cancel()
	via, err := s.modeCtrlParamCtx(ctx, dp.MethodSpotClean, spotParam())
	if err != nil {
		writeJSON(w, http.StatusOK, map[string]any{"ok": false, "error": err.Error()})
		return
	}
	writeJSON(w, http.StatusOK, map[string]any{"ok": true, "command": "spotclean", "bus": via})
}
