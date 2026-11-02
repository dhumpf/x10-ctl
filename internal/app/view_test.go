package app

import (
	"os"
	"path/filepath"
	"testing"
)

// Verbatim from /userdata/ai_map/feat_map_77/map_meta_info.yaml. This is OpenCV
// FileStorage, not YAML: the "%YAML:1.0" and "---" preamble and the inline
// sequence both have to survive the scanner.
const realMapMeta = `%YAML:1.0
---
create time: "1785427803932.658936"
resolution: 5.0000000745058060e-02
width: 266
height: 336
origin: [ -8.9250000000000007e+00, -1.3425000000000001e+01 ]
`

func writeTemp(t *testing.T, name, body string) string {
	t.Helper()
	p := filepath.Join(t.TempDir(), name)
	if err := os.WriteFile(p, []byte(body), 0o600); err != nil {
		t.Fatal(err)
	}
	return p
}

func TestScanRealMapMeta(t *testing.T) {
	p := writeTemp(t, "map_meta_info.yaml", realMapMeta)

	if w, err := scanInt(p, "width"); err != nil || w != 266 {
		t.Errorf("width = %d, %v; want 266", w, err)
	}
	if h, err := scanInt(p, "height"); err != nil || h != 336 {
		t.Errorf("height = %d, %v; want 336", h, err)
	}
	res, err := scanFloat(p, "resolution")
	if err != nil || res < 0.049 || res > 0.051 {
		t.Errorf("resolution = %v, %v; want ~0.05", res, err)
	}
	origin := scanFloats(p, "origin")
	if len(origin) != 2 || origin[0] != -8.925 || origin[1] != -13.425 {
		t.Errorf("origin = %v; want [-8.925 -13.425]", origin)
	}
}

func TestScanIDFile(t *testing.T) {
	// /userdata/ai_map/id.yaml, verbatim
	p := writeTemp(t, "id.yaml", "%YAML:1.0\n---\nid: 77\n")
	if id, err := scanInt(p, "id"); err != nil || id != 77 {
		t.Errorf("id = %d, %v; want 77", id, err)
	}
}

// "height" must not be satisfied by a line for some other key, and a missing
// key must be an error rather than a silent zero - a zero-sized map would
// render as a broken image with no explanation.
func TestScanKeyIsExact(t *testing.T) {
	p := writeTemp(t, "m.yaml", "map_height: 999\nheight: 336\n")
	if h, err := scanInt(p, "height"); err != nil || h != 336 {
		t.Errorf("height = %d, %v; want 336", h, err)
	}
	if _, err := scanInt(p, "width"); err == nil {
		t.Error("missing key returned no error")
	}
	if got := scanFloats(p, "origin"); got != nil {
		t.Errorf("missing sequence returned %v, want nil", got)
	}
}

// BGR to RGB, and the downscale arithmetic. Getting the channel order wrong is
// not a crash, it is a picture that looks almost right - skin goes blue-grey -
// so it needs pinning.
func TestFrameToRGBASwapsChannels(t *testing.T) {
	// 2x1 frame: pixel 0 is pure red stored BGR, pixel 1 is pure blue.
	f := &frame{W: 2, H: 1, Pix: []byte{0x00, 0x00, 0xff, 0xff, 0x00, 0x00}}
	img := f.toRGBA(1)
	if got := img.RGBAAt(0, 0); got.R != 0xff || got.G != 0 || got.B != 0 {
		t.Errorf("pixel 0 = %v, want pure red", got)
	}
	if got := img.RGBAAt(1, 0); got.B != 0xff || got.R != 0 {
		t.Errorf("pixel 1 = %v, want pure blue", got)
	}
	if got := img.RGBAAt(0, 0).A; got != 0xff {
		t.Errorf("alpha = %d, want opaque", got)
	}
}

func TestFrameDownscale(t *testing.T) {
	f := &frame{W: 8, H: 4, Pix: make([]byte, 8*4*3)}
	img := f.toRGBA(2)
	if b := img.Bounds(); b.Dx() != 4 || b.Dy() != 2 {
		t.Errorf("scale 2 gave %v, want 4x2", b)
	}
	// scale 0 must not divide by zero
	if b := f.toRGBA(0).Bounds(); b.Dx() != 8 {
		t.Errorf("scale 0 gave %v, want full size", b)
	}
}
