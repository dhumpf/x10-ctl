package app

import (
	"encoding/json"
	"image"
	"image/color"
	"strings"
	"testing"
)

// rows are y ascending, values are the 8-bit greys the robot writes.
func greyGrid(rows ...[]uint8) *image.Gray {
	img := image.NewGray(image.Rect(0, 0, len(rows[0]), len(rows)))
	for y, row := range rows {
		for x, v := range row {
			img.SetGray(x, y, color.Gray{Y: v})
		}
	}
	return img
}

var fixtureOccupancy = greyGrid(
	[]uint8{0, 0, 0, 0},
	[]uint8{0, 255, 255, 0},
	[]uint8{0, 255, 255, 0},
	[]uint8{125, 125, 125, 125},
)

var fixtureRooms = greyGrid(
	[]uint8{0, 0, 0, 0},
	[]uint8{0, 1, 1, 0},
	[]uint8{0, 0, 2, 0},
	[]uint8{0, 0, 0, 0},
)

// JavaScript rounds half upward, so -0.5 is 0. Go's math.Round gives -1.
func TestJSRoundIsHalfUp(t *testing.T) {
	for _, tc := range []struct {
		in   float64
		want int
	}{{1.5, 2}, {2.5, 3}, {0.5, 1}, {20.5, 21}, {-0.5, 0}, {-1.5, -1}} {
		if got := jsRound(tc.in); got != tc.want {
			t.Errorf("jsRound(%v) = %d, want %d", tc.in, got, tc.want)
		}
	}
}

func TestCompressRunsAreSortedTriples(t *testing.T) {
	runs, dims := compressLayer(fixtureOccupancy, func(v uint8) bool { return v == 0 })
	want := []int{0, 0, 4, 0, 1, 1, 3, 1, 1, 0, 2, 1, 3, 2, 1}
	if len(runs) != len(want) {
		t.Fatalf("runs = %v, want %v", runs, want)
	}
	for i := range want {
		if runs[i] != want[i] {
			t.Fatalf("runs = %v, want %v", runs, want)
		}
	}
	if len(runs)%3 != 0 {
		t.Errorf("compressedPixels length %d is not a multiple of 3", len(runs))
	}
	if dims.PixelCount != 8 {
		t.Errorf("pixelCount = %d, want 8", dims.PixelCount)
	}
	if dims.X.Min != 0 || dims.X.Max != 3 || dims.Y.Min != 0 || dims.Y.Max != 2 {
		t.Errorf("dims = %+v", dims)
	}
}

// A backwards x step silently inflates a run, and only a decompress catches it.
func TestDecompressRoundTrip(t *testing.T) {
	for _, tc := range []struct {
		name string
		keep func(uint8) bool
		img  image.Image
	}{
		{"wall", func(v uint8) bool { return v == 0 }, fixtureOccupancy},
		{"free", func(v uint8) bool { return v == 255 }, fixtureOccupancy},
		{"room1", func(v uint8) bool { return v == 1 }, fixtureRooms},
		{"room2", func(v uint8) bool { return v == 2 }, fixtureRooms},
	} {
		runs, dims := compressLayer(tc.img, tc.keep)
		got := map[[2]int]bool{}
		for i := 0; i+2 < len(runs); i += 3 {
			for k := 0; k < runs[i+2]; k++ {
				got[[2]int{runs[i] + k, runs[i+1]}] = true
			}
		}
		want := map[[2]int]bool{}
		b := tc.img.Bounds()
		for y := b.Min.Y; y < b.Max.Y; y++ {
			for x := b.Min.X; x < b.Max.X; x++ {
				g, _, _, _ := tc.img.At(x, y).RGBA()
				if tc.keep(uint8(g >> 8)) {
					want[[2]int{x - b.Min.X, y - b.Min.Y}] = true
				}
			}
		}
		if len(got) != len(want) || len(got) != dims.PixelCount {
			t.Errorf("%s: decompressed %d pixels, image has %d, dims says %d",
				tc.name, len(got), len(want), dims.PixelCount)
		}
		for p := range want {
			if !got[p] {
				t.Errorf("%s: pixel %v lost in the round trip", tc.name, p)
			}
		}
	}
}

// A nil slice marshals to null and a nil map to null; upstream always emits
// [] and {}. This is the likeliest silent incompatibility in the whole format.
func TestNeverMarshalsNull(t *testing.T) {
	runs, dims := compressLayer(fixtureRooms, func(v uint8) bool { return v == 1 })
	doc := valetudoMap{
		Class: classMap, MetaData: map[string]any{}, Size: valetudoSize{X: 20, Y: 20},
		PixelSize: 5,
		Layers:    []valetudoLayer{newLayer("segment", runs, dims, nil)},
		Entities:  []valetudoEntity{newEntity(classPoint, "charger_location", []int{1, 2})},
	}
	body, err := json.Marshal(doc)
	if err != nil {
		t.Fatal(err)
	}
	if strings.Contains(string(body), "null") {
		t.Errorf("document contains null:\n%s", body)
	}
	for _, k := range []string{`"__class":"ValetudoMap"`, `"pixels":[]`, `"metaData":{`, `"compressedPixels":[`} {
		if !strings.Contains(string(body), k) {
			t.Errorf("missing %s in\n%s", k, body)
		}
	}
}

// An empty layer must still round-trip rather than emitting null dimensions.
func TestEmptyLayerIsStillValid(t *testing.T) {
	runs, dims := compressLayer(fixtureRooms, func(v uint8) bool { return v == 99 })
	l := newLayer("segment", runs, dims, nil)
	body, err := json.Marshal(l)
	if err != nil {
		t.Fatal(err)
	}
	if strings.Contains(string(body), "null") {
		t.Errorf("empty layer marshals null: %s", body)
	}
	if l.Dimensions.PixelCount != 0 {
		t.Errorf("pixelCount = %d, want 0", l.Dimensions.PixelCount)
	}
}

// An arbitrary corner order renders as a bowtie, and the robot reorders the
// corners it stores.
func TestSortRingIsARing(t *testing.T) {
	got := sortRing([][2]int{{0, 0}, {10, 10}, {10, 0}, {0, 10}})
	for i := range got {
		j := (i + 1) % len(got)
		dx, dy := got[j][0]-got[i][0], got[j][1]-got[i][1]
		if dx != 0 && dy != 0 {
			t.Errorf("corner %d -> %d is diagonal, so the quad is crossed: %v", i, j, got)
		}
	}
}

func TestUUIDShape(t *testing.T) {
	u := uuid4()
	if len(u) != 36 || u[14] != '4' {
		t.Errorf("uuid4() = %q", u)
	}
	if u == uuid4() {
		t.Error("uuid4 returned the same value twice")
	}
}
