package app

import (
	"encoding/hex"
	"strings"
	"testing"
)

// The DryCfg the vendor app sent on 2026-08-06 when mop drying was changed from
// the 2 h default to 3 h, read out of the cloud_cli TLS capture:
//
//	12 06 08 01 12 02 08 01   dry { cfg: 1, duration { level: 1 } }
//
// This is the frame dust.go concluded could not be made to work. It can; the
// earlier attempts sent CLOSE, which is 0 and therefore not sent at all.
func TestDryFrameMatchesCapture(t *testing.T) {
	const want = "1206080112020801"

	got := hex.EncodeToString(encodeDry(dryStandard, 1))

	if !strings.Contains(got, want) {
		t.Fatalf("DryCfg body drifted\n got %s\nwant it to contain %s", got, want)
	}
}

// Same container dust.go proved on hardware: request field 2, AutoActionCfg at
// 1, feature inside. Dry is field 2 where collect-dust is 6 and wash is 1.
func TestDryUsesProvenStationContainer(t *testing.T) {
	got := hex.EncodeToString(encodeDry(dryStandard, 1))
	if !strings.HasPrefix(got, "12") {
		t.Fatalf("want StationSetting request field 2 outermost, got %s", got)
	}
	// wash is field 1 of the same container, dry is field 2
	if !strings.Contains(hex.EncodeToString(encodeWash(washModeByTime, 20, 0)), "0a") {
		t.Fatal("wash no longer encodes into field 1")
	}
}

// The duration submessage must be present even at level 0, or a scalar enum of
// 0 vanishes and the write merges as "leave alone".
func TestDryDurationPresentWhenZero(t *testing.T) {
	got := hex.EncodeToString(encodeDry(dryStandard, 0))
	if !strings.Contains(got, "1200") {
		t.Fatalf("duration dropped when zero: %s", got)
	}
}

// Off is 0 and cannot be expressed. Refuse rather than report success and do
// nothing, which is what the earlier CLOSE attempts did.
func TestSetDryRefusesOff(t *testing.T) {
	l := &Local{}
	if err := l.SetDry(nil, dryClose, 1); err == nil {
		t.Fatal("expected cfg 0 to be rejected")
	}
	for _, c := range []struct {
		name       string
		cfg, level int
	}{
		{"cfg too high", 9, 1},
		{"level too high", 1, 9},
		{"level negative", 1, -1},
	} {
		if err := l.SetDry(nil, c.cfg, c.level); err == nil {
			t.Fatalf("%s: expected rejection", c.name)
		}
	}
}

// The manual start/stop the app sends is a different message: DP 173 field 2 is
// ManualActionCmd, and go_dry is its field 2. Recorded so the two are not
// confused - x10-ctl drives that locally via station.Control action 1.
func TestManualDryIsADifferentMessage(t *testing.T) {
	// app start: 04 12 02 10 01  -> {2:{2:1}},  stop: {2:{2:0}}
	// nothing in encodeDry should look like that
	got := hex.EncodeToString(encodeDry(dryStandard, 1))
	if strings.Contains(got, "12021001") {
		t.Fatalf("dry setting frame collided with the manual go_dry command: %s", got)
	}
}
