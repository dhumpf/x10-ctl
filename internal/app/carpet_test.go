package app

import (
	"encoding/hex"
	"strings"
	"testing"
)

// CleanParam field 2 is clean_carpet_strategy.
//
// Do not confuse it with corner_clean, which is field 2 of MopMode, and MopMode
// is CleanParam field 4. Two different field 2s at different depths, which is
// exactly the confusion this test exists to prevent.
//
// Setting carpet handling to avoidance in the vendor app on 2026-08-06 sent,
// as DP 154:
//
//	0a 06  12 02 08 01  2a 00      f1 { f2 { f1: 1 }, f5 {} }
//
// and user_config then read clean_carpet_strategy 1, with corner_clean and
// clean_carpet_mode both still 0. Setting it back to vacuum-only sent
// f2 {} - empty, because 0 is the proto3 default.
func TestCarpetStrategyIsCleanParamFieldTwo(t *testing.T) {
	one := 1
	got := hex.EncodeToString(cleanParam(&CleanSettings{CarpetStrategy: &one}))
	if got != "12020801" {
		t.Fatalf("carpet strategy 1 should encode f2{f1:1}\n got %s\nwant 12020801", got)
	}
}

// Zero is the default and must go out as an empty submessage, which is what the
// app sent when carpet went back to vacuum-only.
func TestCarpetStrategyZeroIsEmptySubmessage(t *testing.T) {
	zero := 0
	got := hex.EncodeToString(cleanParam(&CleanSettings{CarpetStrategy: &zero}))
	if got != "1200" {
		t.Fatalf("carpet strategy 0 should encode f2{}\n got %s\nwant 1200", got)
	}
}

// Unset means no change: the field must not appear at all, or every write of
// some other parameter would quietly reset carpet handling.
func TestCarpetStrategyAbsentWhenUnset(t *testing.T) {
	four := 4
	got := hex.EncodeToString(cleanParam(&CleanSettings{Fan: &four}))
	if strings.Contains(got, "12") && strings.HasPrefix(got, "12") {
		t.Fatalf("field 2 emitted when carpet was not requested: %s", got)
	}
}

// The whole DP 154 frame the app sent, reproduced from the two fields it
// carried: carpet on, smart mode off.
func TestCarpetFrameMatchesCapture(t *testing.T) {
	one, off := 1, false
	got := hex.EncodeToString(cleanParam(&CleanSettings{
		CarpetStrategy: &one, SmartMode: &off,
	}))
	const want = "12020801" + "2a00"
	if got != want {
		t.Fatalf("DP 154 CleanParam drifted\n got %s\nwant %s", got, want)
	}
}
