package app

import (
	"encoding/hex"
	"strings"
	"testing"
)

// The exact field-22 block the vendor app sent on 2026-08-06 with the window
// set to 01:17 to 16:43:
//
//	0a 02 08 01          Switch    { value: 1 }
//	12 04 08 01 10 11    TimePoint { hour: 1,  minute: 17 }
//	1a 04 08 10 10 2b    TimePoint { hour: 16, minute: 43 }
//
// user_config read back peak_charge_switch true, 1:17 and 16:43, so both ends
// agree. If this stops matching, the encoder has drifted from the only frame
// anyone has seen.
func TestPeakChargeFrameMatchesCapture(t *testing.T) {
	const want = "0a020801" + "1204080110" + "11" + "1a04081010" + "2b"

	got := hex.EncodeToString(encodePeakCharge(&PeakCharge{
		Enabled: true, StartHour: 1, StartMinute: 17, EndHour: 16, EndMinute: 43,
	}))

	if !strings.Contains(got, want) {
		t.Fatalf("peak charge body drifted\n got %s\nwant it to contain %s", got, want)
	}
}

// Hours are 24-hour. 4 pm arrived as 16, which is the whole reason the test
// window used an afternoon end time.
func TestPeakChargeIsTwentyFourHour(t *testing.T) {
	got := hex.EncodeToString(encodePeakCharge(&PeakCharge{
		Enabled: true, EndHour: 16, EndMinute: 43,
	}))
	if !strings.Contains(got, "0810") { // hour = 0x10 = 16
		t.Fatalf("end hour 16 not encoded as 0x10: %s", got)
	}
	if strings.Contains(got, "080410") { // would be hour = 4
		t.Fatalf("end hour encoded as 4, not 16: %s", got)
	}
}

// It rides in Unisetting, request field 22, one past high_detection at 21.
func TestPeakChargeUsesUnisettingField22(t *testing.T) {
	got := hex.EncodeToString(encodePeakCharge(&PeakCharge{Enabled: true}))
	// field 2 wrapper, then field 22 as a two-byte varint tag b2 01
	if !strings.HasPrefix(got, "12") {
		t.Fatalf("want Unisetting request field 2 outermost: %s", got)
	}
	if !strings.Contains(got, "b201") {
		t.Fatalf("want field 22 tag b2 01: %s", got)
	}
}

// Switch is a message, so off is an empty submessage and not an absent one -
// presence is the signal, as in the switch block.
func TestPeakChargeOffIsPresentButEmpty(t *testing.T) {
	got := hex.EncodeToString(encodePeakCharge(&PeakCharge{Enabled: false}))
	if !strings.Contains(got, "0a00") {
		t.Fatalf("disabled switch should be an empty submessage: %s", got)
	}
}

func TestSetPeakChargeRejectsOutOfRange(t *testing.T) {
	l := &Local{}
	for _, p := range []PeakCharge{
		{StartHour: 24}, {StartMinute: 60}, {EndHour: -1}, {EndMinute: 99},
	} {
		if err := l.SetPeakCharge(nil, &p); err == nil {
			t.Fatalf("expected rejection for %+v", p)
		}
	}
}
