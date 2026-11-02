package app

import (
	"image"
	"image/color"
	"testing"
)

// The default layer is the room map. It answers "which room is that", which is
// the question the page is usually open for.
func TestDefaultLayerIsRooms(t *testing.T) {
	l, ok := layerByKey("")
	if !ok || l.Key != "house" {
		t.Fatalf("default layer is %q", l.Key)
	}
	if mapLayers[0].Key != "house" || mapLayers[1].Key != "map" {
		t.Fatalf("order is %s then %s", mapLayers[0].Key, mapLayers[1].Key)
	}
	if _, ok := layerByKey("nope"); ok {
		t.Error("unknown layer should be rejected")
	}
}

func grey(vals ...uint8) image.Image {
	img := image.NewGray(image.Rect(0, 0, len(vals), 1))
	for i, v := range vals {
		img.SetGray(i, 0, color.Gray{Y: v})
	}
	return img
}

// Carpet is value 80 in an otherwise ordinary occupancy grid. Rendered as plain
// greyscale it is a mid-grey nobody can pick out, which is why the carpet layer
// looked identical to the occupancy one.
func TestCarpetLayerRecoloursOnlyTheCarpet(t *testing.T) {
	out := renderMapLayer(grey(0, 125, 255, carpetValue), "carpet", "")
	for i, want := range []color.RGBA{
		{0, 0, 0, 0xff},
		{125, 125, 125, 0xff},
		{255, 255, 255, 0xff},
		carpetIDColours[1],
	} {
		got := out.At(i, 0).(color.RGBA)
		if got != want {
			t.Errorf("pixel %d: got %v want %v", i, got, want)
		}
	}
	if carpetIDColours[1] == (color.RGBA{carpetValue, carpetValue, carpetValue, 0xff}) {
		t.Error("carpet has to differ from the grey it replaces")
	}
}

// The id map numbers carpets rather than marking them, and this robot has both
// 1 and 2. They must not render the same, and the higher id must be darker.
func TestCarpetIDsAreDistinctAndDarken(t *testing.T) {
	out := renderMapLayer(grey(0, 1, 2), "carpetid", "")
	zero := out.At(0, 0).(color.RGBA)
	one := out.At(1, 0).(color.RGBA)
	two := out.At(2, 0).(color.RGBA)
	if zero.A != 0 {
		t.Error("0 is not carpet and should be transparent")
	}
	if one == two {
		t.Fatal("carpet 1 and carpet 2 render the same")
	}
	lum := func(c color.RGBA) int { return int(c.R) + int(c.G) + int(c.B) }
	if lum(two) >= lum(one) {
		t.Errorf("carpet 2 should be darker: %v vs %v", two, one)
	}
}

// An out-of-range id must still draw as carpet rather than falling off the end
// of the palette.
func TestCarpetIDOutOfRange(t *testing.T) {
	out := renderMapLayer(grey(99), "carpetid", "")
	if got := out.At(0, 0).(color.RGBA); got != carpetIDColours[len(carpetIDColours)-1] {
		t.Fatalf("got %v", got)
	}
}

func TestRoomsPaletteUnchanged(t *testing.T) {
	out := renderMapLayer(grey(0, 1, 8), "rooms", "")
	if got := out.At(0, 0).(color.RGBA); got.A != 0 {
		t.Error("room 0 is not a room and should be transparent")
	}
	if got := out.At(1, 0).(color.RGBA); got != roomColours[1] {
		t.Errorf("room 1: %v", got)
	}
	if got := out.At(2, 0).(color.RGBA); got != roomColours[8] {
		t.Errorf("room 8: %v", got)
	}
}

// A mask layer is greyscale and must not pick up a palette by accident.
func TestMaskLayerStaysGrey(t *testing.T) {
	out := renderMapLayer(grey(0, 255), "mask", "")
	if got := out.At(0, 0).(color.RGBA); got != (color.RGBA{0, 0, 0, 0xff}) {
		t.Errorf("got %v", got)
	}
	if got := out.At(1, 0).(color.RGBA); got != (color.RGBA{255, 255, 255, 0xff}) {
		t.Errorf("got %v", got)
	}
}
