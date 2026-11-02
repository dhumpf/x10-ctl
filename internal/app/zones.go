package app

// Keep-out zones: virtual walls, forbidden zones and no-mop zones.
//
//	MapEditRequest { RestrictedZone restricted_zone = 8 }   // method 4
//	RestrictedZone { repeated Line       virtual_walls   = 1;
//	                 repeated Quadrangle forbidden_zones = 2;
//	                 repeated Quadrangle ban_mop_zones   = 3;
//	                 uint32 map_id = 4; uint32 releases = 5 }
//	Line       { Point p0 = 1; Point p1 = 2 }
//	Quadrangle { Point p0 = 1; Point p1 = 2; Point p2 = 3; Point p3 = 4 }
//	Point      { sint32 x = 1; sint32 y = 2 }   // zigzag, see fieldSint
//
// The robot persists these as world metres, in OpenCV matrices:
//
//	forbid_zones.yaml   8 doubles per zone, four (x, y) corners
//	forbid_lines.yaml   4 doubles per wall, two endpoints
//
// but Point is an integer type, so the wire cannot be metres. It is
// CENTIMETRES, measured rather than guessed: re-sending the robot's own zone as
// millimetres moved it by a factor of a hundred - 950 went in and 9.5 m came
// back - which pins the scale exactly.
//
// That round trip is worth keeping as the test for this file. Re-sending the
// zones already present should leave forbid_zones.yaml unchanged; if the units
// are ever wrong again the zone jumps and the file says so immediately.
//
// The robot also reorders the corners it stores, so a byte-identical file is
// too strong a check - compare the set of corners, not their order.

import (
	"bufio"
	"context"
	"fmt"
	"github.com/dhumpf/x10-ctl/internal/pb"
	"math"
	"net/http"
	"os"
	"path/filepath"
	"strconv"
	"strings"
	"time"
)

// ZonePoint is one corner in world metres.
type ZonePoint struct {
	X float64 `json:"x"`
	Y float64 `json:"y"`
}

// Zone is a keep-out area. Walls carry two points, zones four.
type Zone struct {
	Kind   string      `json:"kind"` // wall, forbid, nomop
	Points []ZonePoint `json:"points"`
}

// readOpenCVMatrix pulls the `data: [ ... ]` block out of an OpenCV FileStorage
// matrix. The values run across several lines, so this is not a scanKey job.
func readOpenCVMatrix(path string) ([]float64, error) {
	f, err := os.Open(path)
	if err != nil {
		return nil, err
	}
	defer f.Close()

	var body strings.Builder
	inData := false
	sc := bufio.NewScanner(f)
	for sc.Scan() {
		line := sc.Text()
		if !inData {
			i := strings.Index(line, "data:")
			if i < 0 {
				continue
			}
			inData = true
			line = line[i+len("data:"):]
		}
		body.WriteString(line)
		if strings.Contains(line, "]") {
			break
		}
	}
	if !inData {
		return nil, fmt.Errorf("no data block in %s", path)
	}
	s := body.String()
	s = strings.Trim(strings.TrimSpace(s), "[]")
	if strings.TrimSpace(s) == "" {
		return nil, nil
	}
	var out []float64
	for _, part := range strings.Split(s, ",") {
		part = strings.TrimSpace(part)
		if part == "" {
			continue
		}
		v, err := strconv.ParseFloat(part, 64)
		if err != nil {
			return nil, fmt.Errorf("%s: %q: %w", path, part, err)
		}
		out = append(out, v)
	}
	return out, nil
}

func zonesFromMatrix(vals []float64, stride int, kind string) []Zone {
	var out []Zone
	for i := 0; i+stride <= len(vals); i += stride {
		z := Zone{Kind: kind}
		for j := 0; j+1 < stride; j += 2 {
			z.Points = append(z.Points, ZonePoint{X: vals[i+j], Y: vals[i+j+1]})
		}
		out = append(out, z)
	}
	return out
}

// readZones reads the robot's persisted keep-out geometry.
func readZones() ([]Zone, error) {
	meta, err := readMapMeta()
	if err != nil {
		return nil, err
	}
	var zones []Zone
	if v, err := readOpenCVMatrix(filepath.Join(meta.Dir, "forbid_zones.yaml")); err == nil {
		zones = append(zones, zonesFromMatrix(v, 8, "forbid")...)
	}
	if v, err := readOpenCVMatrix(filepath.Join(meta.Dir, "forbid_lines.yaml")); err == nil {
		zones = append(zones, zonesFromMatrix(v, 4, "wall")...)
	}
	return zones, nil
}

// cmPoint encodes one Point in centimetres, rounding rather than truncating so
// a corner does not creep by up to a centimetre on every round trip.
func cmPoint(num uint64, p ZonePoint) []byte {
	body := pb.FieldSint(1, int64(math.Round(p.X*100)))
	body = append(body, pb.FieldSint(2, int64(math.Round(p.Y*100)))...)
	return pb.FieldBytes(num, body)
}

// SetZones replaces the whole keep-out set.
//
// Replaces, not appends: RestrictedZone carries repeated fields with no
// identity, so the only way to remove one is to send everything else. Callers
// must pass the full set they want to keep.
func (l *Local) SetZones(ctx context.Context, seq uint64, mapID int, zones []Zone) error {
	var body []byte
	for _, z := range zones {
		var pts []byte
		for i, p := range z.Points {
			pts = append(pts, cmPoint(uint64(i+1), p)...)
		}
		switch z.Kind {
		case "wall":
			if len(z.Points) != 2 {
				return fmt.Errorf("a virtual wall needs two points, got %d", len(z.Points))
			}
			body = append(body, pb.FieldBytes(1, pts)...)
		case "forbid":
			if len(z.Points) != 4 {
				return fmt.Errorf("a forbidden zone needs four points, got %d", len(z.Points))
			}
			body = append(body, pb.FieldBytes(2, pts)...)
		case "nomop":
			if len(z.Points) != 4 {
				return fmt.Errorf("a no-mop zone needs four points, got %d", len(z.Points))
			}
			body = append(body, pb.FieldBytes(3, pts)...)
		default:
			return fmt.Errorf("unknown zone kind %q", z.Kind)
		}
	}
	if mapID > 0 {
		body = append(body, pb.FieldVarint(4, uint64(mapID))...)
	}
	return l.mapEdit(ctx, mapEditRestricted, seq, mapID, pb.FieldBytes(8, body))
}

func (s *server) handleZones(w http.ResponseWriter, r *http.Request) {
	if r.Method == http.MethodGet {
		zones, err := readZones()
		if err != nil {
			writeJSON(w, http.StatusServiceUnavailable, map[string]any{"ok": false, "error": err.Error()})
			return
		}
		frame, _ := readMapFrame()
		writeJSON(w, http.StatusOK, map[string]any{"ok": true, "zones": zones, "frame": frame})
		return
	}
	if r.Method != http.MethodPost {
		writeJSON(w, http.StatusMethodNotAllowed, map[string]string{"error": "GET or POST"})
		return
	}

	var body struct {
		Zones []Zone `json:"zones"`
	}
	if err := decodeJSON(w, r, &body); err != nil {
		writeJSON(w, http.StatusBadRequest, map[string]any{"ok": false, "error": "bad json"})
		return
	}
	ctx, cancel := context.WithTimeout(r.Context(), 20*time.Second)
	defer cancel()
	mapID := 0
	if v, err := readMapMeta(); err == nil {
		mapID = v.ID
	}
	if err := s.local.SetZones(ctx, s.nextSeq(), mapID, body.Zones); err != nil {
		writeJSON(w, http.StatusOK, map[string]any{"ok": false, "error": err.Error()})
		return
	}
	select {
	case <-ctx.Done():
	case <-time.After(1500 * time.Millisecond):
	}
	zones, _ := readZones()
	writeJSON(w, http.StatusOK, map[string]any{"ok": true, "bus": "local", "zones": zones})
}
