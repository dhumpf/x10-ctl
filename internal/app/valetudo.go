package app

// The ValetudoMap wire format. Pure functions over images and numbers, no disk,
// so the golden test is real coverage away from the robot.

import (
	"crypto/rand"
	"fmt"
	"image"
	"math"
	"sort"
)

// __class is load-bearing: upstream DESERIALIZE dispatches on it.
const (
	classMap     = "ValetudoMap"
	classLayer   = "MapLayer"
	classPoint   = "PointMapEntity"
	classPolygon = "PolygonMapEntity"
	classLine    = "LineMapEntity"
)

type valetudoMap struct {
	Class     string           `json:"__class"`
	MetaData  map[string]any   `json:"metaData"`
	Size      valetudoSize     `json:"size"`
	PixelSize int              `json:"pixelSize"`
	Layers    []valetudoLayer  `json:"layers"`
	Entities  []valetudoEntity `json:"entities"`
}

// Centimetres, not pixels. The document mixes both units and they are not
// interchangeable: layer runs are pixel indices, entity points are centimetres.
type valetudoSize struct {
	X int `json:"x"`
	Y int `json:"y"`
}

type valetudoLayer struct {
	Class            string         `json:"__class"`
	MetaData         map[string]any `json:"metaData"`
	Type             string         `json:"type"`
	Pixels           []int          `json:"pixels"`
	Dimensions       valetudoDims   `json:"dimensions"`
	CompressedPixels []int          `json:"compressedPixels"`
}

type valetudoAxis struct {
	Min int `json:"min"`
	Max int `json:"max"`
	Mid int `json:"mid"`
	Avg int `json:"avg"`
}

type valetudoDims struct {
	X          valetudoAxis `json:"x"`
	Y          valetudoAxis `json:"y"`
	PixelCount int          `json:"pixelCount"`
}

// The four upstream entity classes serialise identically and differ only in
// __class and the legal type strings, so one struct is honest.
type valetudoEntity struct {
	Class    string         `json:"__class"`
	MetaData map[string]any `json:"metaData"`
	Points   []int          `json:"points"`
	Type     string         `json:"type"`
}

// jsRound is JavaScript's Math.round: half away from zero upward, so -0.5 is 0
// rather than -1. Go's math.Round rounds half away from zero in both directions.
func jsRound(v float64) int {
	return int(math.Floor(v + 0.5))
}

// compressLayer walks the image row by row and emits [xStart, y, count] triples
// for every horizontal run the predicate accepts.
func compressLayer(img image.Image, keep func(v uint8) bool) (runs []int, dims valetudoDims) {
	runs = []int{}
	b := img.Bounds()
	minX, minY := math.MaxInt32, math.MaxInt32
	maxX, maxY := -1, -1
	var sumX, sumY, count int

	for y := b.Min.Y; y < b.Max.Y; y++ {
		runStart := -1
		for x := b.Min.X; x <= b.Max.X; x++ {
			in := false
			if x < b.Max.X {
				g, _, _, _ := img.At(x, y).RGBA()
				in = keep(uint8(g >> 8))
			}
			switch {
			case in && runStart < 0:
				runStart = x
			case !in && runStart >= 0:
				n := x - runStart
				runs = append(runs, runStart-b.Min.X, y-b.Min.Y, n)
				for i := 0; i < n; i++ {
					sumX += runStart - b.Min.X + i
				}
				sumY += (y - b.Min.Y) * n
				count += n
				if runStart-b.Min.X < minX {
					minX = runStart - b.Min.X
				}
				if x-1-b.Min.X > maxX {
					maxX = x - 1 - b.Min.X
				}
				if y-b.Min.Y < minY {
					minY = y - b.Min.Y
				}
				if y-b.Min.Y > maxY {
					maxY = y - b.Min.Y
				}
				runStart = -1
			}
		}
	}
	if count == 0 {
		return runs, valetudoDims{}
	}
	dims = valetudoDims{
		X:          valetudoAxis{Min: minX, Max: maxX, Mid: jsRound(float64(minX+maxX) / 2), Avg: jsRound(float64(sumX) / float64(count))},
		Y:          valetudoAxis{Min: minY, Max: maxY, Mid: jsRound(float64(minY+maxY) / 2), Avg: jsRound(float64(sumY) / float64(count))},
		PixelCount: count,
	}
	return runs, dims
}

// newLayer builds one layer. Slices and maps are non-nil throughout: a nil slice
// marshals to null and upstream always emits [] and {}.
func newLayer(kind string, runs []int, dims valetudoDims, meta map[string]any) valetudoLayer {
	if meta == nil {
		meta = map[string]any{}
	}
	if runs == nil {
		runs = []int{}
	}
	meta["area"] = dims.PixelCount * 25
	return valetudoLayer{
		Class: classLayer, MetaData: meta, Type: kind,
		Pixels: []int{}, Dimensions: dims, CompressedPixels: runs,
	}
}

func newEntity(class, kind string, points []int) valetudoEntity {
	if points == nil {
		points = []int{}
	}
	return valetudoEntity{Class: class, MetaData: map[string]any{}, Points: points, Type: kind}
}

// sortRing orders polygon corners by angle about their centroid. The robot
// reorders the corners it stores, and an arbitrary order renders as a bowtie.
func sortRing(pts [][2]int) [][2]int {
	if len(pts) < 3 {
		return pts
	}
	var cx, cy float64
	for _, p := range pts {
		cx += float64(p[0])
		cy += float64(p[1])
	}
	cx /= float64(len(pts))
	cy /= float64(len(pts))
	out := append([][2]int(nil), pts...)
	sort.Slice(out, func(i, j int) bool {
		return math.Atan2(float64(out[i][1])-cy, float64(out[i][0])-cx) <
			math.Atan2(float64(out[j][1])-cy, float64(out[j][0])-cx)
	})
	return out
}

// uuid4 for metaData.nonce. Ten lines beats a dependency in a binary that has
// none at all.
func uuid4() string {
	var b [16]byte
	if _, err := rand.Read(b[:]); err != nil {
		return "00000000-0000-4000-8000-000000000000"
	}
	b[6] = (b[6] & 0x0f) | 0x40
	b[8] = (b[8] & 0x3f) | 0x80
	return fmt.Sprintf("%x-%x-%x-%x-%x", b[0:4], b[4:6], b[6:8], b[8:10], b[10:16])
}
