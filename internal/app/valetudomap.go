package app

// Building our map into ValetudoMap and serving it from /api/map. The vendor
// block keeps everything Valetudo has no type for, so the UI loses nothing.

import (
	"bytes"
	"fmt"
	"image"
	"image/color"
	"image/png"
	"net/http"
	"os"
	"path/filepath"
	"sort"
	"strconv"
)

// Occupancy values the robot writes into map.png.
const (
	occObstacle = 0
	occUnknown  = 125
	occFree     = 255
)

func loadLayerImage(dir, file string) (image.Image, error) {
	b, err := os.ReadFile(filepath.Join(dir, file))
	if err != nil {
		return nil, err
	}
	return png.Decode(bytes.NewReader(b))
}

// pixelSizeOf is the grid in centimetres. The stored resolution is a widened
// float32, so res*100 misses 5 by 7.45e-08 and a tight epsilon rejects it.
func pixelSizeOf(meta *mapMeta) (int, bool) {
	cm := meta.Resolution * 100
	n := jsRound(cm)
	return n, n > 0 && cm-float64(n) < 1e-4 && float64(n)-cm < 1e-4
}

// worldToCM converts world metres to the document's centimetres. No axis flips:
// pose.go established by measurement that image y runs with world y, and the
// horizontal flip in view.go is a render-time choice the UI mirrors separately.
func worldToCM(meta *mapMeta, px int, wx, wy float64) (int, int) {
	cx := jsRound((wx - meta.Origin[0]) / meta.Resolution * float64(px))
	cy := jsRound((wy - meta.Origin[1]) / meta.Resolution * float64(px))
	return cx, cy
}

// mirrorX returns the image mirrored left to right with pixel values intact.
//
// The stored grid is mirrored against the actual home - view.go serves the PNG
// with flip=h by default for exactly that reason, settled by eye. The UI mirrors
// at render time; Valetudo does not, and has no flip of its own, so the document
// has to arrive the right way round.
func mirrorX(src image.Image) image.Image {
	b := src.Bounds()
	w, h := b.Dx(), b.Dy()
	dst := image.NewGray(image.Rect(0, 0, w, h))
	for y := 0; y < h; y++ {
		for x := 0; x < w; x++ {
			g, _, _, _ := src.At(b.Min.X+w-1-x, b.Min.Y+y).RGBA()
			dst.SetGray(x, y, color.Gray{Y: uint8(g >> 8)})
		}
	}
	return dst
}

// docMirrorX mirrors a document x coordinate. Its own inverse, and applied on
// the way out and again on the way in, so what reaches the robot is unchanged.
func docMirrorX(sizeX, x int) int { return sizeX - x }

type clampCount struct{ n int }

func (c *clampCount) clamp(v, max int) int {
	if v < 0 {
		c.n++
		return 0
	}
	if v > max {
		c.n++
		return max
	}
	return v
}

// buildValetudoMap assembles the document. Errors on the optional pieces are
// dropped rather than failing the map: a missing dock pose is not a broken map.
func (s *server) buildValetudoMap(meta *mapMeta) (*valetudoMap, error) {
	px, ok := pixelSizeOf(meta)
	if !ok {
		return nil, fmt.Errorf("map resolution %g m/px is not a whole number of centimetres", meta.Resolution)
	}

	occ, err := loadLayerImage(meta.Dir, "map.png")
	if err != nil {
		return nil, err
	}
	rooms, roomsErr := loadLayerImage(meta.Dir, "house.png")

	doc := &valetudoMap{
		Class:     classMap,
		MetaData:  map[string]any{},
		Size:      valetudoSize{X: meta.Width * px, Y: meta.Height * px},
		PixelSize: px,
		Layers:    []valetudoLayer{},
		Entities:  []valetudoEntity{},
	}

	// A free pixel belongs to a segment or to floor, never both, so floor is
	// the free pixels that no room claims.
	inRoom := func(x, y int) bool {
		if roomsErr != nil {
			return false
		}
		b := rooms.Bounds()
		if x >= b.Dx() || y >= b.Dy() {
			return false
		}
		g, _, _, _ := rooms.At(b.Min.X+x, b.Min.Y+y).RGBA()
		return uint8(g>>8) != 0
	}

	// Mirror once here so every layer and its bounds come out already correct;
	// flipping runs by hand gets the run start and the dimensions wrong.
	occ, rooms = mirrorX(occ), mirrorX(rooms)

	floorRuns, floorDims := subtractRooms(occ, occ.Bounds(), inRoom)
	doc.Layers = append(doc.Layers, newLayer("floor", floorRuns, floorDims, nil))

	wallRuns, wallDims := compressLayer(occ, func(v uint8) bool { return v == occObstacle })
	doc.Layers = append(doc.Layers, newLayer("wall", wallRuns, wallDims, nil))

	_, unknownDims := compressLayer(occ, func(v uint8) bool { return v == occUnknown })

	names := map[int]string{}
	if list, _, err := readRooms(); err == nil {
		for _, r := range list {
			names[r.ID] = r.Name
		}
	}

	dropped := map[string]any{"unknown": unknownDims.PixelCount}
	if roomsErr == nil {
		for _, id := range roomIDsPresent(rooms) {
			want := uint8(id)
			runs, dims := compressLayer(rooms, func(v uint8) bool { return v == want })
			if dims.PixelCount == 0 {
				continue
			}
			// segmentId is the house.png pixel value, which is the same room id
			// roomsParam and HA clean_segments use. One id space, no table.
			m := map[string]any{"segmentId": strconv.Itoa(id), "active": false}
			if n, ok := names[id]; ok && n != "" {
				m["name"] = n
			}
			doc.Layers = append(doc.Layers, newLayer("segment", runs, dims, m))
		}
	} else {
		dropped["house_png"] = roomsErr.Error()
	}

	var cc clampCount
	maxX, maxY := doc.Size.X-1, doc.Size.Y-1
	pt := func(wx, wy float64) []int {
		x, y := worldToCM(meta, px, wx, wy)
		return []int{cc.clamp(docMirrorX(doc.Size.X, x), maxX), cc.clamp(y, maxY)}
	}

	frame := &MapFrame{Resolution: meta.Resolution, Width: meta.Width, Height: meta.Height,
		OriginX: meta.Origin[0], OriginY: meta.Origin[1]}
	if dock, err := readDockPose(frame); err == nil && dock != nil {
		doc.Entities = append(doc.Entities, newEntity(classPoint, "charger_location", pt(dock.X, dock.Y)))
	}
	if pose, ok := s.local.Pose(); ok && pose != nil && (pose.Frame == "map" || pose.Frame == "map~odom") {
		doc.Entities = append(doc.Entities, newEntity(classPoint, "robot_position", pt(pose.X, pose.Y)))
	}

	if zones, err := readZones(); err == nil {
		for _, z := range zones {
			var ring [][2]int
			for _, p := range z.Points {
				q := pt(p.X, p.Y)
				ring = append(ring, [2]int{q[0], q[1]})
			}
			switch z.Kind {
			case "forbid":
				ring = sortRing(ring)
				flat := []int{}
				for _, p := range ring {
					flat = append(flat, p[0], p[1])
				}
				doc.Entities = append(doc.Entities, newEntity(classPolygon, "no_go_area", flat))
			case "wall":
				flat := []int{}
				for _, p := range ring {
					flat = append(flat, p[0], p[1])
				}
				doc.Entities = append(doc.Entities, newEntity(classLine, "virtual_wall", flat))
			}
		}
	}

	total := 0
	for _, l := range doc.Layers {
		total += l.Dimensions.PixelCount * px * px
	}
	doc.MetaData = map[string]any{
		"nonce":          uuid4(),
		"totalLayerArea": total,
		"vendorMapId":    meta.ID,
		"version":        2,
		"x10Frame": map[string]any{
			"width": meta.Width, "height": meta.Height,
			"resolution_m_per_px": meta.Resolution, "origin_m": meta.Origin, "dir": meta.Dir,
		},
		"x10Layers":  describeLayers(meta),
		"x10Dropped": dropped,
		"x10Clamped": cc.n,
		"x10Note":    "read-only. no_mop_area is not readable on this robot",
	}
	return doc, nil
}

// subtractRooms builds the floor layer: free pixels that no segment claims.
func subtractRooms(occ image.Image, b image.Rectangle, inRoom func(x, y int) bool) ([]int, valetudoDims) {
	masked := image.NewGray(image.Rect(0, 0, b.Dx(), b.Dy()))
	for y := 0; y < b.Dy(); y++ {
		for x := 0; x < b.Dx(); x++ {
			g, _, _, _ := occ.At(b.Min.X+x, b.Min.Y+y).RGBA()
			if uint8(g>>8) == occFree && !inRoom(x, y) {
				masked.SetGray(x, y, color.Gray{Y: occFree})
			}
		}
	}
	return compressLayer(masked, func(v uint8) bool { return v == occFree })
}

// roomIDsPresent lists the distinct non-zero values in house.png, ascending, so
// the layer order is deterministic and the golden test is stable.
func roomIDsPresent(img image.Image) []int {
	seen := map[int]bool{}
	b := img.Bounds()
	for y := b.Min.Y; y < b.Max.Y; y++ {
		for x := b.Min.X; x < b.Max.X; x++ {
			g, _, _, _ := img.At(x, y).RGBA()
			if v := int(uint8(g >> 8)); v != 0 {
				seen[v] = true
			}
		}
	}
	out := make([]int, 0, len(seen))
	for v := range seen {
		out = append(out, v)
	}
	sort.Ints(out)
	return out
}

func (s *server) handleValetudoMap(w http.ResponseWriter, r *http.Request) {
	meta, err := readMapMeta()
	if err != nil {
		writeJSON(w, http.StatusServiceUnavailable, map[string]any{"ok": false, "error": err.Error()})
		return
	}
	doc, err := s.buildValetudoMap(meta)
	if err != nil {
		writeJSON(w, http.StatusServiceUnavailable, map[string]any{"ok": false, "error": err.Error()})
		return
	}
	writeJSON(w, http.StatusOK, doc)
}
