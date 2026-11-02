package app

import (
	"encoding/hex"
	"strings"
	"testing"
)

// The WashCfg the vendor app sent on 2026-08-06 when auto-wash was set to
// "every 20 minutes", read out of the cloud_cli TLS capture:
//
//	f1 { f1: 1, f3 { f1: 20 } }   by time, 20
//	f2 {}                          wash_duration, present and empty
//	f3: 1                          cfg = STANDARD
//
// user_config confirmed it as backwash_freq_mode: 1, backwash_freq_interval: 20.
// If this stops matching, the encoder has drifted from the only frame we have
// ever seen the app produce.
func TestWashFrameMatchesCapture(t *testing.T) {
	const wantInner = "0a06080" + "11a020814" + "1200" + "1801"

	got := hex.EncodeToString(encodeWash(washModeByTime, 20, 0))

	if !strings.Contains(got, wantInner) {
		t.Fatalf("WashCfg body drifted\n got %s\nwant it to contain %s", got, wantInner)
	}
}

// The station container is the one dust.go proved on hardware: request field 2,
// AutoActionCfg at 1, and the per-feature config inside that. Wash is field 1
// where collect-dust is field 6.
func TestWashUsesProvenStationContainer(t *testing.T) {
	got := hex.EncodeToString(encodeWash(washModeByTime, 20, 0))
	if !strings.HasPrefix(got, "12") {
		t.Fatalf("want StationSetting request field 2 outermost, got %s", got)
	}

	dust := hex.EncodeToString(encodeCollectDust(false))
	if !strings.HasPrefix(dust, "12") {
		t.Fatalf("dust frame no longer starts at field 2: %s", dust)
	}
}

// wash_duration must be emitted even when it is zero. A scalar enum at 0 is
// proto3's default and would be dropped, so the empty submessage is the only
// way to say SHORT - the same reason a false Switch survives in dust.go.
func TestWashDurationPresentWhenZero(t *testing.T) {
	got := hex.EncodeToString(encodeWash(washModeByTime, 20, 0))
	if !strings.Contains(got, "1200") {
		t.Fatalf("wash_duration dropped when zero: %s", got)
	}

	withLevel := hex.EncodeToString(encodeWash(washModeByTime, 20, 2))
	if !strings.Contains(withLevel, "12020802") {
		t.Fatalf("wash_duration level 2 not encoded: %s", withLevel)
	}
}

// Omitting the interval must leave the robot's value alone rather than send 0.
func TestWashOmitsUnsetInterval(t *testing.T) {
	got := hex.EncodeToString(encodeWash(washModeByTime, 0, 0))
	if strings.Contains(got, "1a02") {
		t.Fatalf("interval encoded when unset: %s", got)
	}
}

func TestSetWashRejectsOutOfRange(t *testing.T) {
	l := &Local{}
	for _, c := range []struct {
		name                     string
		mode, interval, duration int
	}{
		{"mode high", 9, 20, 0},
		{"mode negative", -1, 20, 0},
		{"duration high", 1, 20, 9},
		{"interval negative", 1, -5, 0},
		{"interval absurd", 1, 100000, 0},
	} {
		if err := l.SetWash(nil, c.mode, c.interval, c.duration); err == nil {
			t.Fatalf("%s: expected rejection", c.name)
		}
	}
}
