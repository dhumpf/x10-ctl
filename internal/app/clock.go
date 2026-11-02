package app

// Time, without a cloud.
//
// The robot has no NTP client - no ntpd, no chrony, nothing in init.d - and
// gets its clock from the cloud protocol: cloud_cli receives a `cloud_time`
// response and calls settimeofday. That is fine while a broker is reachable and
// useless for a robot dropped in a room with no uplink, which is the case this
// exists for.
//
// There is a hardware clock at /dev/rtc0 and it ticks, but nothing writes it.
// It was found reading exactly twelve hours off, and that turns out not to be
// drift: busybox hwclock on this build mishandles the timezone flags, and
// `hwclock -w -l` writes a value twelve hours out. Measured, on the robot:
//
//	hwclock -w -u   register := system local wall time     round-trips
//	hwclock -w -l   register := twelve hours off           the original damage
//	hwclock -r      prints the register verbatim
//	hwclock -r -l   prints the register plus the TZ offset
//
// So the register holds local time, `-w -u` and a bare `-r` are the only pair
// that agree, and everything here uses them. Reading the register as anything
// other than local time is how it ends up looking twelve hours wrong again.
//
// The remaining question is where good time comes from with no network at all,
// and the answer is the phone in your hand: a browser knows the time, and the
// page can post it. No NTP, no cloud, no internet - just the device you are
// already using to drive the robot.

import (
	"context"
	"fmt"
	"log"
	"net/http"
	"strconv"
	"strings"
	"time"
)

// plausibleAfter is the floor for "the clock looks set". Anything earlier means
// the system booted without a usable clock. Chosen as a date comfortably before
// this code existed and comfortably after the epoch.
var plausibleAfter = time.Date(2025, 1, 1, 0, 0, 0, 0, time.UTC)

// readRTC returns the hardware clock. The register holds local time; see above.
func (l *Local) readRTC(ctx context.Context) (time.Time, error) {
	out, err := l.run(ctx, "hwclock -r")
	if err != nil {
		return time.Time{}, fmt.Errorf("hwclock -r: %w: %s", err, strings.TrimSpace(string(out)))
	}
	// "Thu Jul 30 13:35:24 2026  0.000000 seconds"
	s := strings.TrimSpace(string(out))
	if i := strings.Index(s, "  "); i > 0 {
		s = s[:i]
	}
	t, err := time.ParseInLocation("Mon Jan _2 15:04:05 2006", s, time.Local)
	if err != nil {
		return time.Time{}, fmt.Errorf("hwclock output %q: %w", s, err)
	}
	return t, nil
}

// writeRTC copies the system clock into the hardware clock.
func (l *Local) writeRTC(ctx context.Context) error {
	out, err := l.run(ctx, "hwclock -w -u")
	if err != nil {
		return fmt.Errorf("hwclock -w: %w: %s", err, strings.TrimSpace(string(out)))
	}
	return nil
}

// rawRTC reads the register itself, out of sysfs, with no hwclock flags in the
// way. The kernel exposes exactly what is stored; what it MEANS is the open
// question - see readRTCReport.
func (l *Local) rawRTC(ctx context.Context) (time.Time, error) {
	out, err := l.run(ctx, "cat /sys/class/rtc/rtc0/date /sys/class/rtc/rtc0/time")
	if err != nil {
		return time.Time{}, fmt.Errorf("sysfs rtc: %w", err)
	}
	f := strings.Fields(strings.TrimSpace(string(out)))
	if len(f) != 2 {
		return time.Time{}, fmt.Errorf("sysfs rtc: unexpected %q", strings.TrimSpace(string(out)))
	}
	return time.ParseInLocation("2006-01-02 15:04:05", f[0]+" "+f[1], time.UTC)
}

// readRTCReport describes the register both ways rather than picking one.
//
// There are two writers with different conventions. cloud_cli opens /dev/rtc0
// and calls settimeofday in its cloud_time path, and the value it leaves is
// local wall time in a field the kernel treats as UTC. Writing it with
// `hwclock -w -u` stores true UTC instead. Whichever is written last wins, so
// a single "drift" number is a claim this code is not entitled to make.
func (l *Local) readRTCReport(ctx context.Context) map[string]any {
	raw, err := l.rawRTC(ctx)
	if err != nil {
		return map[string]any{"error": err.Error()}
	}
	now := time.Now()
	_, offset := now.Zone()
	asUTC := raw
	asLocal := raw.Add(-time.Duration(offset) * time.Second)
	out := map[string]any{
		"raw":                    raw.Format("2006-01-02T15:04:05") + "Z (register as stored)",
		"if_utc_delta_seconds":   int(now.Sub(asUTC).Round(time.Second).Seconds()),
		"if_local_delta_seconds": int(now.Sub(asLocal).Round(time.Second).Seconds()),
	}
	// Whichever reading lands within a minute of now is the convention the last
	// writer used. Neither matching means the register is genuinely stale.
	switch {
	case abs(out["if_utc_delta_seconds"].(int)) < 60:
		out["convention"] = "utc"
	case abs(out["if_local_delta_seconds"].(int)) < 60:
		out["convention"] = "local wall time"
	default:
		out["convention"] = "neither - stale"
	}
	return out
}

func abs(n int) int {
	if n < 0 {
		return -n
	}
	return n
}

// SetTime sets the system clock and persists it to the RTC.
//
// `date -s` rather than a syscall: busybox date is already on the robot, and
// the alternative is a raw settimeofday that would need its own arm64 syscall
// wrapper for one call.
func (l *Local) SetTime(ctx context.Context, t time.Time) error {
	if !l.Available() {
		return fmt.Errorf("not running on the robot")
	}
	if t.Before(plausibleAfter) {
		return fmt.Errorf("refusing to set the clock to %s", t.Format(time.RFC3339))
	}
	arg := t.UTC().Format("2006-01-02 15:04:05")
	out, err := l.run(ctx, "date -u -s '"+arg+"'")
	if err != nil {
		return fmt.Errorf("date -s: %w: %s", err, strings.TrimSpace(string(out)))
	}
	// Best effort: a system clock that is right until the next reboot still
	// beats refusing the whole operation because the RTC would not take.
	if err := l.writeRTC(ctx); err != nil {
		return fmt.Errorf("clock set, but RTC not written: %w", err)
	}
	return nil
}

// RestoreClock seeds the system clock from the RTC when it looks unset.
//
// Runs at startup. Does nothing in the normal case - the kernel already seeds
// from the RTC on boot - and matters when that did not happen or when the
// system clock has been left at the epoch by something else. Schedules are
// wall-clock, so a robot that thinks it is 1970 quietly never cleans.
func (l *Local) RestoreClock() {
	if !l.Available() {
		return
	}
	if time.Now().After(plausibleAfter) {
		return
	}
	ctx, cancel := context.WithTimeout(context.Background(), 10*time.Second)
	defer cancel()
	rtc, err := l.readRTC(ctx)
	if err != nil {
		log.Printf("clock: system time looks unset and the RTC is unreadable: %v", err)
		return
	}
	if rtc.Before(plausibleAfter) {
		log.Printf("clock: both the system clock and the RTC look unset (RTC says %s); "+
			"set the time from the web UI", rtc.Format(time.RFC3339))
		return
	}
	if err := l.SetTime(ctx, rtc); err != nil {
		log.Printf("clock: restoring from RTC: %v", err)
		return
	}
	log.Printf("clock: restored from RTC to %s", rtc.Format(time.RFC3339))
}

func (s *server) handleTime(w http.ResponseWriter, r *http.Request) {
	ctx, cancel := context.WithTimeout(r.Context(), 10*time.Second)
	defer cancel()

	if r.Method == http.MethodPost {
		ms, err := strconv.ParseInt(r.URL.Query().Get("epoch_ms"), 10, 64)
		if err != nil {
			writeJSON(w, http.StatusBadRequest, map[string]any{
				"ok": false, "error": "epoch_ms is required",
			})
			return
		}
		if err := s.local.SetTime(ctx, time.UnixMilli(ms)); err != nil {
			writeJSON(w, http.StatusOK, map[string]any{"ok": false, "error": err.Error()})
			return
		}
		writeJSON(w, http.StatusOK, map[string]any{
			"ok": true, "now": time.Now().Format(time.RFC3339), "rtc_written": true,
		})
		return
	}

	now := time.Now()
	out := map[string]any{
		"ok":        true,
		"system":    now.Format(time.RFC3339),
		"epoch_ms":  now.UnixMilli(),
		"plausible": now.After(plausibleAfter),
		// No NTP anywhere on the robot; the vendor path is a cloud round trip.
		"source": "rtc or last set",
	}
	if rtc, err := s.local.readRTC(ctx); err == nil {
		out["rtc"] = rtc.Format(time.RFC3339)
	}
	// Both readings, and which convention the last writer used. A single drift
	// figure read as 21600 for a register that was simply written by the other
	// writer, and 21600 is the timezone offset, not an error.
	out["rtc_register"] = s.local.readRTCReport(ctx)
	writeJSON(w, http.StatusOK, out)
}
