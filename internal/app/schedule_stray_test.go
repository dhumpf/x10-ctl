package app

import (
	"os"
	"path/filepath"
	"testing"
)

// A backup copy left in the schedule dir is not inert. robotapp loads every
// file it finds there, so schedule_clean_2.yaml.pre-timertest-20260817 - the
// July record, 17:30 on bits 36 - fired on Tue and Fri alongside the real
// 19:00 slot for five days. x10-ctl's own listing hid it: the old filter took
// anything ending in .yaml, which the backups do not.
func scheduleFixture(t *testing.T, files map[string]string) {
	t.Helper()
	dir := t.TempDir()
	for name, body := range files {
		if err := os.WriteFile(filepath.Join(dir, name), []byte(body), 0o600); err != nil {
			t.Fatal(err)
		}
	}
	old, oldQ := scheduleDir, scheduleQuarantine
	scheduleDir, scheduleQuarantine = dir, filepath.Join(t.TempDir(), "backups")
	t.Cleanup(func() { scheduleDir, scheduleQuarantine = old, oldQ })
}

const slot1 = "schedule_id: 1\nstatus_valid: 1\nstatus_opened: 1\n" +
	"desc_timing_hours: 9\ndesc_timing_minutes: 0\ndesc_cycle_week_bits: 42\n"

const strayJuly = "schedule_id: 2\nstatus_valid: 1\nstatus_opened: 1\n" +
	"desc_timing_hours: 17\ndesc_timing_minutes: 30\ndesc_cycle_week_bits: 36\n"

func TestReadSchedulesIgnoresBackupCopies(t *testing.T) {
	scheduleFixture(t, map[string]string{
		"schedule_clean_1.yaml":                        slot1,
		"schedule_clean_2.yaml.pre-timertest-20260817": strayJuly,
		"schedule_clean_1.yaml.pre-bitprobe-20260827":  slot1,
	})

	got, err := readSchedules()
	if err != nil {
		t.Fatal(err)
	}
	if len(got) != 1 || got[0].ID != 1 || got[0].At != "09:00" {
		t.Fatalf("want only slot 1 at 09:00, got %+v", got)
	}
}

func TestStrayScheduleFilesNamesTheBackups(t *testing.T) {
	scheduleFixture(t, map[string]string{
		"schedule_clean_1.yaml":                        slot1,
		"schedule_clean_2.yaml.pre-timertest-20260817": strayJuly,
	})

	strays, err := strayScheduleFiles()
	if err != nil {
		t.Fatal(err)
	}
	if len(strays) != 1 || strays[0] != "schedule_clean_2.yaml.pre-timertest-20260817" {
		t.Fatalf("want the backup copy listed, got %v", strays)
	}
}

func TestQuarantineMovesStraysAndKeepsSlots(t *testing.T) {
	scheduleFixture(t, map[string]string{
		"schedule_clean_1.yaml":                        slot1,
		"schedule_clean_2.yaml.pre-timertest-20260817": strayJuly,
	})

	moved, err := quarantineStraySchedules()
	if err != nil {
		t.Fatal(err)
	}
	if len(moved) != 1 {
		t.Fatalf("want 1 file moved, got %v", moved)
	}
	if _, err := os.Stat(filepath.Join(scheduleDir, moved[0])); !os.IsNotExist(err) {
		t.Fatalf("stray still in the schedule dir: %v", err)
	}
	if _, err := os.Stat(filepath.Join(scheduleQuarantine, moved[0])); err != nil {
		t.Fatalf("stray not in quarantine: %v", err)
	}
	if _, err := os.Stat(filepath.Join(scheduleDir, "schedule_clean_1.yaml")); err != nil {
		t.Fatalf("slot file moved: %v", err)
	}
	again, err := quarantineStraySchedules()
	if err != nil || len(again) != 0 {
		t.Fatalf("second run should be a no-op, got %v %v", again, err)
	}
}
