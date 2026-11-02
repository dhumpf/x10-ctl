package app

// Rendering the two things the robot knows that no protocol ever exposed: what
// it can see, and the floor plan it has built.
//
// Both are read-only and neither touches a control channel. The camera comes
// out of shared memory (camera_linux_arm64.go); the map is simply files on
// disk, which is the part that took longest to establish - the map does exist
// on the akos bus and on an mds channel, but neither publishes anything while
// the robot is docked, and cloud_cli logs "pixel buff empty" every two seconds
// the whole time. The files are always there.

import (
	"bufio"
	"bytes"
	"context"
	"fmt"
	"image"
	"image/color"

	"image/jpeg"
	"image/png"
	"log"
	"net/http"
	"os"
	"path/filepath"
	"strconv"
	"strings"
	"time"
)

const mapRoot = "/userdata/ai_map"

// toRGBA converts a BGR frame to an image, downscaling by an integer factor.
//
// Nearest neighbour on purpose: this runs on the robot's own CPU alongside
// everything else, a full frame is 6.2 MB, and the result is a preview in a
// phone browser. Interpolating would cost more than it shows.
func (f *frame) toRGBA(scale int) *image.RGBA {
	if scale < 1 {
		scale = 1
	}
	w, h := f.W/scale, f.H/scale
	img := image.NewRGBA(image.Rect(0, 0, w, h))
	for y := 0; y < h; y++ {
		srcRow := (y * scale) * f.W * 3
		dstRow := y * img.Stride
		for x := 0; x < w; x++ {
			i := srcRow + (x*scale)*3
			d := dstRow + x*4
			// stored BGR, hence the reversal - see camera_linux_arm64.go
			img.Pix[d+0] = f.Pix[i+2]
			img.Pix[d+1] = f.Pix[i+1]
			img.Pix[d+2] = f.Pix[i+0]
			img.Pix[d+3] = 0xff
		}
	}
	return img
}

// frameAge returns how long ago the frame in shared memory was written.
//
// The stamp is CLOCK_MONOTONIC seconds, which is what /proc/uptime counts, so
// the difference is the age. This matters more than it looks: camera_svc stops
// writing when its pipeline is not running, and a stale frame is
// indistinguishable from a live one by content alone. Without this the UI shows
// a three-minute-old picture of the room and calls it a camera.
func frameAge(stamp float64) (float64, bool) {
	b, err := os.ReadFile("/proc/uptime")
	if err != nil {
		return 0, false
	}
	f := strings.Fields(string(b))
	if len(f) == 0 {
		return 0, false
	}
	up, err := strconv.ParseFloat(f[0], 64)
	if err != nil {
		return 0, false
	}
	return up - stamp, true
}

// handleCameraMeta reports the frame's age without encoding a JPEG.
func (s *server) handleCameraMeta(w http.ResponseWriter, r *http.Request) {
	if !cameraAvailable() {
		writeJSON(w, http.StatusOK, map[string]any{"available": false})
		return
	}
	s.holdCamera(r)
	f, err := grabFrame()
	if err != nil {
		writeJSON(w, http.StatusOK, map[string]any{"available": false, "error": err.Error()})
		return
	}
	out := map[string]any{"available": true, "width": f.W, "height": f.H}
	if age, ok := frameAge(f.Stamp); ok {
		out["age_seconds"] = int(age)
		out["live"] = age < 5
	}
	writeJSON(w, http.StatusOK, out)
}

// handleCamera serves one JPEG snapshot.
func (s *server) handleCamera(w http.ResponseWriter, r *http.Request) {
	if !cameraAvailable() {
		writeJSON(w, http.StatusServiceUnavailable, map[string]any{
			"ok": false, "error": "no camera segment (x10-ctl must run on the robot, and camera_svc must be up)",
		})
		return
	}
	s.holdCamera(r)
	f, err := grabFrame()
	if err != nil {
		writeJSON(w, http.StatusServiceUnavailable, map[string]any{"ok": false, "error": err.Error()})
		return
	}
	scale := atoiDefault(r.URL.Query().Get("scale"), 4)
	if scale < 1 || scale > 16 {
		scale = 4
	}
	quality := atoiDefault(r.URL.Query().Get("q"), 70)
	if quality < 10 || quality > 95 {
		quality = 70
	}
	w.Header().Set("Content-Type", "image/jpeg")
	w.Header().Set("Cache-Control", "no-store")
	if age, ok := frameAge(f.Stamp); ok {
		w.Header().Set("X-Frame-Age", strconv.Itoa(int(age)))
	}
	if err := jpeg.Encode(w, f.toRGBA(scale), &jpeg.Options{Quality: quality}); err != nil {
		log.Printf("camera: encode: %v", err)
	}
}

// --- map

type mapMeta struct {
	ID         int       `json:"id"`
	Width      int       `json:"width"`
	Height     int       `json:"height"`
	Resolution float64   `json:"resolution_m_per_px"`
	Origin     []float64 `json:"origin_m"`
	Dir        string    `json:"dir"`
}

// currentMapID reads the id of the live map. The files are OpenCV FileStorage,
// which opens with "%YAML:1.0" and "---" and is otherwise close enough to
// key: value that a scanner beats pulling in a YAML parser.
func currentMapID() (int, error) {
	return scanInt(filepath.Join(mapRoot, "id.yaml"), "id")
}

func readMapMeta() (*mapMeta, error) {
	id, err := currentMapID()
	if err != nil {
		return nil, err
	}
	dir := filepath.Join(mapRoot, fmt.Sprintf("feat_map_%d", id))
	m := &mapMeta{ID: id, Dir: dir}
	m.Width, _ = scanInt(filepath.Join(dir, "map_meta_info.yaml"), "width")
	m.Height, _ = scanInt(filepath.Join(dir, "map_meta_info.yaml"), "height")
	m.Resolution, _ = scanFloat(filepath.Join(dir, "map_meta_info.yaml"), "resolution")
	m.Origin = scanFloats(filepath.Join(dir, "map_meta_info.yaml"), "origin")
	return m, nil
}

// mapLayer is one drawable file in the map directory.
//
// meta names the layer's OWN metadata file. Only three layers share the main
// map's frame; cliff is 172x229 and coverage is 174x232 against the map's
// 266x336, each with its own origin, so nothing drawn in map pixels lines up on
// them. That is what `main` records and why the overlay is suppressed elsewhere.
type mapLayer struct {
	Key   string `json:"key"`
	Label string `json:"label"`
	File  string `json:"-"`
	Meta  string `json:"-"`
	Kind  string `json:"kind"`
	Main  bool   `json:"main_frame"`
}

// mapLayers is the served set, in the order the UI shows them.
//
// Rooms first. It is the one that answers "which room is that" and it was
// second only because the occupancy grid was written first.
var mapLayers = []mapLayer{
	{"house", "Rooms", "house.png", "map_meta_info.yaml", "rooms", true},
	{"map", "Occupancy", "map.png", "map_meta_info.yaml", "occupancy", true},
	{"carpet", "Carpet", "carpet_obstacle_map.png", "map_meta_info.yaml", "carpet", true},
	{"carpetid", "Carpet ids", "id_filled_carpet_map.png", "id_filled_carpet_map_meta_info.yaml", "carpetid", false},
	{"cliff", "Cliff", "cliff_map.png", "cliff_map_meta_info.yaml", "mask", false},
	{"coverage", "Coverage", "coverage_range_map.png", "coverage_range_map_meta_info.yaml", "mask", false},
}

func layerByKey(key string) (mapLayer, bool) {
	if key == "" {
		key = "house"
	}
	for _, l := range mapLayers {
		if l.Key == key {
			return l, true
		}
	}
	return mapLayer{}, false
}

// handleMap serves a map layer as the PNG it already is on disk.
//
//	rooms      house.png, room ids 1-8 as raw values, palette applied here
//	occupancy  map.png, 0 obstacle, 125 unknown, 255 free
//	carpet     the occupancy grid again, plus 80 where carpet was detected
//	carpetid   which carpet is which, values 1 and 2, its own smaller frame
//	mask       cliff and coverage, binary, each in its own frame
func (s *server) handleMap(w http.ResponseWriter, r *http.Request) {
	meta, err := readMapMeta()
	if err != nil {
		writeJSON(w, http.StatusServiceUnavailable, map[string]any{
			"ok": false, "error": fmt.Sprintf("no map on disk: %v", err),
		})
		return
	}
	layer, ok := layerByKey(r.URL.Query().Get("layer"))
	if !ok {
		writeJSON(w, http.StatusBadRequest, map[string]any{"ok": false, "error": "unknown layer"})
		return
	}
	file := layer.File
	b, err := os.ReadFile(filepath.Join(meta.Dir, file))
	if err != nil {
		writeJSON(w, http.StatusNotFound, map[string]any{"ok": false, "error": err.Error()})
		return
	}

	src, err := png.Decode(bytes.NewReader(b))
	if err != nil {
		writeJSON(w, http.StatusInternalServerError, map[string]any{"ok": false, "error": err.Error()})
		return
	}

	// flip: h by default, settled by eye against the actual home. A vertical
	// flip was the obvious guess - occupancy grids put the origin bottom-left
	// while PNG row 0 is the top - but it was wrong; the stored image is
	// mirrored left to right relative to the room it describes. The UI exposes
	// all four so this can be re-checked rather than argued about.
	flip := r.URL.Query().Get("flip")
	if flip == "" {
		flip = "h"
	}
	out := renderMapLayer(src, layer.Kind, flip)

	w.Header().Set("Content-Type", "image/png")
	w.Header().Set("Cache-Control", "no-store")
	if err := png.Encode(w, out); err != nil {
		log.Printf("map: encode: %v", err)
	}
}

// roomColours give each room id its own hue. house.png stores ids as raw
// values 1-8 in an 8-bit greyscale image, so served untouched it is black on
// black - every pixel is within 8/255 of zero.
var roomColours = []color.RGBA{
	{0x00, 0x00, 0x00, 0x00}, // 0: not a room, transparent
	{0x4c, 0x9f, 0xff, 0xff},
	{0xff, 0x8c, 0x42, 0xff},
	{0x5a, 0xd6, 0x8a, 0xff},
	{0xe0, 0x6c, 0xd8, 0xff},
	{0xf2, 0xd0, 0x4b, 0xff},
	{0x6b, 0xe0, 0xd8, 0xff},
	{0xb0, 0x80, 0xff, 0xff},
	{0xa0, 0xa8, 0xb4, 0xff},
}

// carpetValue is the pixel the carpet detector writes into the occupancy grid.
//
// Measured, not guessed: carpet_obstacle_map.png is byte-for-byte map.png -
// 0 obstacle, 125 unknown, 255 free - with one extra value present, 80, on 618
// pixels. Served as plain greyscale that is a mid-grey between unknown and
// obstacle and cannot be picked out by eye, which is why the carpet layer used
// to look identical to the occupancy one.
const carpetValue = 80

// carpetIDColours index by the value in id_filled_carpet_map.png, which numbers
// the carpets rather than marking them. Darker as the id rises, so two carpets
// in one room stay apart.
var carpetIDColours = []color.RGBA{
	{0x00, 0x00, 0x00, 0x00}, // 0: not carpet
	{0x9a, 0x6b, 0x3f, 0xff},
	{0x5e, 0x3f, 0x22, 0xff},
	{0x3a, 0x27, 0x14, 0xff},
	{0x24, 0x18, 0x0c, 0xff},
}

// renderMapLayer applies the flip and whatever palette the layer needs.
func renderMapLayer(src image.Image, kind, flip string) image.Image {
	b := src.Bounds()
	w, h := b.Dx(), b.Dy()
	dst := image.NewRGBA(image.Rect(0, 0, w, h))
	for y := 0; y < h; y++ {
		for x := 0; x < w; x++ {
			sx, sy := x, y
			if flip == "v" || flip == "both" {
				sy = h - 1 - y
			}
			if flip == "h" || flip == "both" {
				sx = w - 1 - x
			}
			g, _, _, _ := src.At(b.Min.X+sx, b.Min.Y+sy).RGBA()
			v := uint8(g >> 8)
			switch kind {
			case "rooms":
				if int(v) < len(roomColours) {
					dst.Set(x, y, roomColours[v])
				} else {
					dst.Set(x, y, color.RGBA{0xff, 0xff, 0xff, 0xff})
				}
			case "carpet":
				// The grid underneath stays readable; only the carpet is
				// recoloured, because the point is where the carpet is in the
				// room rather than the carpet on its own.
				if v == carpetValue {
					dst.Set(x, y, carpetIDColours[1])
				} else {
					dst.Set(x, y, color.RGBA{v, v, v, 0xff})
				}
			case "carpetid":
				if int(v) < len(carpetIDColours) {
					dst.Set(x, y, carpetIDColours[v])
				} else {
					dst.Set(x, y, carpetIDColours[len(carpetIDColours)-1])
				}
			default:
				dst.Set(x, y, color.RGBA{v, v, v, 0xff})
			}
		}
	}
	return dst
}

func (s *server) handleMapMeta(w http.ResponseWriter, r *http.Request) {
	meta, err := readMapMeta()
	if err != nil {
		writeJSON(w, http.StatusServiceUnavailable, map[string]any{"ok": false, "error": err.Error()})
		return
	}
	writeJSON(w, http.StatusOK, map[string]any{
		"id": meta.ID, "width": meta.Width, "height": meta.Height,
		"resolution_m_per_px": meta.Resolution, "origin_m": meta.Origin,
		"dir": meta.Dir, "layers": describeLayers(meta),
	})
}

// describeLayers reports each layer that is actually on disk, with its own
// frame. A layer whose frame differs from the map's cannot carry the dock, the
// robot or a keep-out zone, and the UI needs to know that rather than drawing
// them a hundred pixels out.
func describeLayers(meta *mapMeta) []map[string]any {
	out := []map[string]any{}
	for _, l := range mapLayers {
		if _, err := os.Stat(filepath.Join(meta.Dir, l.File)); err != nil {
			continue
		}
		d := map[string]any{"key": l.Key, "label": l.Label, "kind": l.Kind}
		w, errW := scanInt(filepath.Join(meta.Dir, l.Meta), "width")
		h, errH := scanInt(filepath.Join(meta.Dir, l.Meta), "height")
		res, errR := scanFloat(filepath.Join(meta.Dir, l.Meta), "resolution")
		origin := scanFloats(filepath.Join(meta.Dir, l.Meta), "origin")
		if errW == nil && errH == nil {
			d["width"], d["height"] = w, h
		}
		if errR == nil {
			d["resolution_m_per_px"] = res
		}
		if len(origin) == 2 {
			d["origin_m"] = origin
		}
		// Trust the file, not the table: a layer is in the map frame only if
		// its own metadata says the same size and origin.
		d["main_frame"] = w == meta.Width && h == meta.Height &&
			len(origin) == 2 && len(meta.Origin) == 2 &&
			origin[0] == meta.Origin[0] && origin[1] == meta.Origin[1]
		out = append(out, d)
	}
	return out
}

// --- tiny OpenCV-FileStorage readers

func scanKey(path, key string) (string, bool) {
	f, err := os.Open(path)
	if err != nil {
		return "", false
	}
	defer f.Close()
	sc := bufio.NewScanner(f)
	for sc.Scan() {
		line := strings.TrimSpace(sc.Text())
		if !strings.HasPrefix(line, key+":") {
			continue
		}
		return strings.TrimSpace(strings.TrimPrefix(line, key+":")), true
	}
	return "", false
}

func scanInt(path, key string) (int, error) {
	v, ok := scanKey(path, key)
	if !ok {
		return 0, fmt.Errorf("%s: no %q", filepath.Base(path), key)
	}
	n, err := strconv.Atoi(strings.Trim(v, `"`))
	if err != nil {
		return 0, fmt.Errorf("%s: %q is not an int", filepath.Base(path), key)
	}
	return n, nil
}

func scanFloat(path, key string) (float64, error) {
	v, ok := scanKey(path, key)
	if !ok {
		return 0, fmt.Errorf("no %q", key)
	}
	return strconv.ParseFloat(strings.Trim(v, `"`), 64)
}

// scanFloats reads an OpenCV inline sequence: origin: [ -8.925e+00, -1.34e+01 ]
func scanFloats(path, key string) []float64 {
	v, ok := scanKey(path, key)
	if !ok {
		return nil
	}
	v = strings.Trim(v, "[] ")
	var out []float64
	for _, part := range strings.Split(v, ",") {
		if f, err := strconv.ParseFloat(strings.TrimSpace(part), 64); err == nil {
			out = append(out, f)
		}
	}
	return out
}

func atoiDefault(s string, def int) int {
	if n, err := strconv.Atoi(s); err == nil {
		return n
	}
	return def
}

// handleLidar returns one sweep as JSON for the page to draw.
//
// Powers the laser on the way in. The robot cuts laser power whenever it is
// parked, so without this the segment holds whatever the last task left there -
// which is how an earlier version managed to render a stale sweep and call it
// live.
func (s *server) handleLidar(w http.ResponseWriter, r *http.Request) {
	ctx, cancel := context.WithTimeout(r.Context(), 12*time.Second)
	defer cancel()

	if r.URL.Query().Get("power") == "off" {
		if err := s.laser.Release(ctx); err != nil {
			writeJSON(w, http.StatusOK, map[string]any{"ok": false, "error": err.Error()})
			return
		}
		writeJSON(w, http.StatusOK, map[string]any{"ok": true, "powered": false})
		return
	}

	if _, err := s.laser.Touch(ctx); err != nil {
		writeJSON(w, http.StatusServiceUnavailable, map[string]any{
			"ok": false, "error": "laser power: " + err.Error(),
		})
		return
	}
	sc, err := s.local.Scan(ctx)
	if err != nil {
		writeJSON(w, http.StatusServiceUnavailable, map[string]any{"ok": false, "error": err.Error()})
		return
	}
	writeJSON(w, http.StatusOK, map[string]any{"ok": true, "powered": true, "scan": sc})
}

// holdCamera asks camera_svc to keep the sensor streaming.
//
// Best effort and deliberately not fatal: the frame in shared memory is worth
// serving even if the switch failed, and a stale one is still better than an
// error page. The age the caller gets back tells them which they have.
func (s *server) holdCamera(r *http.Request) {
	if s.camera == nil || !s.local.Available() {
		return
	}
	ctx, cancel := context.WithTimeout(r.Context(), 4*time.Second)
	defer cancel()
	if _, err := s.camera.Touch(ctx); err != nil {
		log.Printf("camera: stream hold: %v", err)
	}
}

// handleCameraLight sets the supplement LED, 0..100.
//
// Independent of the stream hold: you may want light without polling frames,
// and the release path below turns it off with the stream so the LED cannot be
// left burning by a closed tab.
func (s *server) handleCameraLight(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPost {
		writeJSON(w, http.StatusMethodNotAllowed, map[string]string{"error": "POST only"})
		return
	}
	level := atoiDefault(r.URL.Query().Get("level"), -1)
	if level < 0 || level > 100 {
		writeJSON(w, http.StatusBadRequest, map[string]any{"ok": false, "error": "level must be 0..100"})
		return
	}
	ctx, cancel := context.WithTimeout(r.Context(), 8*time.Second)
	defer cancel()
	if err := s.local.CameraLight(ctx, level); err != nil {
		writeJSON(w, http.StatusOK, map[string]any{"ok": false, "error": err.Error()})
		return
	}
	s.setLight(level)
	writeJSON(w, http.StatusOK, map[string]any{"ok": true, "level": level})
}
