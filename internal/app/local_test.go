package app

import (
	"encoding/json"
	"testing"
)

// The exact line akostopic printed on 2026-07-30. If the parser stops handling
// this, the battery silently stops updating and the UI just shows a stale value,
// so it is worth pinning verbatim.
const realBatteryLine = `[2026-07-30 14:21:19.536] /topic/chassis/batteryinfo[node_sensor_msgs/BatteryInfo]->>[31]{"header":{"seq":4791,"stamp":{"sec":9604,"nsec":223542786},"frame_id":""},"charge_state":0,"battery_state":0,"peripheral_state":0,"battery_level":100,"battery_vol":16404,"charging_chip_error":0,"opening_error":0,"charge_error":0,"discharge_error":0}`

func TestParseAkosBatteryLine(t *testing.T) {
	raw, ok := parseAkosJSON(realBatteryLine)
	if !ok {
		t.Fatal("did not find a payload in a real akostopic line")
	}
	var b batteryInfo
	if err := json.Unmarshal(raw, &b); err != nil {
		t.Fatalf("payload is not valid JSON: %v", err)
	}
	if b.BatteryLevel != 100 {
		t.Errorf("battery_level = %d, want 100", b.BatteryLevel)
	}
	if b.BatteryVol != 16404 {
		t.Errorf("battery_vol = %d, want 16404", b.BatteryVol)
	}
	// akostopic renders proto bools as 0/1. Decoding these as Go bool fails the
	// whole message, and a skipped message means a battery that never updates -
	// which is exactly what this test caught.
	if b.ChargeError != 0 || b.DischargeError != 0 || b.ChargingChipError != 0 {
		t.Error("fault flags set on a healthy sample")
	}
}

// akostopic prints a shared-memory attach banner before any message, and a
// trailing "---" separator between them. Neither carries "->>", and treating
// one as a message would log a parse error every two seconds.
func TestParseAkosSkipsNonMessages(t *testing.T) {
	for _, line := range []string{
		"Try to malloc share memory of 16777600 bytes... ",
		"Attach to share memory [key= 1880162305,shmId 24] succeed.",
		`akostopic AKOS_DOMAIN_ID: "AA:BB:CC:00:11:22"`,
		"---",
		"",
		"Wait a moment...",
	} {
		if _, ok := parseAkosJSON(line); ok {
			t.Errorf("treated a banner line as a message: %q", line)
		}
	}
}

// A line with the marker but no object must not be reported as parsed.
func TestParseAkosRejectsMarkerWithoutJSON(t *testing.T) {
	if _, ok := parseAkosJSON("[ts] /topic/x[std_msgs/Empty]->>[0]"); ok {
		t.Error("accepted a message with no JSON body")
	}
}

// Off the robot every entry point must degrade quietly rather than shelling out.
func TestLocalUnavailableIsInert(t *testing.T) {
	l := &Local{} // available=false, the off-device zero value
	if l.Available() {
		t.Fatal("zero value must not claim availability")
	}
	if err := l.Beep(t.Context(), true); err == nil {
		t.Error("Beep off-device returned nil; it must not look like it worked")
	}
	if b := l.Battery(); b.Valid {
		t.Error("battery must be invalid before any sample")
	}
	st := l.Status()
	if st["available"] != false {
		t.Errorf("status = %v, want available:false", st)
	}
	// must not panic or block
	stop := make(chan struct{})
	close(stop)
	l.WatchBattery(stop)
}

// The channel stops publishing when a run ends and mdsNewest then returns the
// last record for ever, so state has to be gated on the header sequence.
func TestRecordSeqIsTheFreshnessSignal(t *testing.T) {
	// header{ stamp{...}, node, seq }, body{ work_state }
	rec := []byte{0x0a, 0x03, 0x18, 0xb8, 0x54, 0x12, 0x02, 0x10, 0x05}
	seq, ok := recordSeq(rec)
	if !ok {
		t.Fatal("no seq in the header")
	}
	if seq != 10808 {
		t.Errorf("seq = %d, want 10808", seq)
	}
	if _, ok := recordSeq([]byte{0x12, 0x02, 0x10, 0x05}); ok {
		t.Error("a record with no header reported a seq")
	}
}
