package app

// Consumables and maintenance counters. Read-only.
//
// user_config.yaml carries two numbers per part:
//
//	side_brush_run_time: 13380
//	maintenance_side_brush_duration: 13380
//
// Which is which is not obvious from the names, and the file answers it. Every
// pair on this robot is equal except the dust bag, which reads 1549 against
// 1297, and the dust bag is the only part that has been used - the same file
// records 163 m2 cleaned in total against 110 m2 at the last bag change. So the
// number that moved is the maintenance_* one, which makes `*_run_time` the full
// life of the part and `maintenance_*_duration` what is left of it. Reported
// both ways round below so a reader can check rather than take that on trust.
//
// The unit is minutes. 13380 is 223 hours, which is the side-brush life these
// robots are sold with; seconds would make it under four hours and the mop
// under an hour.
//
// There is a eros.proto.msg.app.ConsumableSetting channel, which is presumably
// how a reset is sent. Nothing here writes it: a reset that fires by accident
// cannot be undone, and the counters are worth reading long before they are
// worth clearing.

import (
	"fmt"
	"net/http"
	"sort"
)

// consumable is one part. total is the *_run_time key, left is the
// maintenance_*_duration key; either may be absent.
//
// paired says the two keys are the same part, which is only claimed where the
// names say so - side_brush_run_time against maintenance_side_brush_duration
// and so on. Two pairings looked obvious and are not: scrape_run_time reads
// 13380 like the brushes while maintenance_mid_drum_mop_wiper_duration reads
// 9240 like the station parts, and every genuine pair on an unused robot reads
// the same number twice. Pairing those anyway produced a confident "69% left"
// for a part that has never been used. They are still reported, as two numbers
// and no percentage, which is what the file actually supports.
type consumable struct {
	Name   string `json:"name"`
	Total  string `json:"-"`
	Left   string `json:"-"`
	Where  string `json:"where"`
	paired bool
}

var consumables = []consumable{
	{"side brush", "side_brush_run_time", "maintenance_side_brush_duration", "robot", true},
	{"rolling brush", "rolling_brush_run_time", "maintenance_rolling_brush_duration", "robot", true},
	{"filter mesh", "filter_mesh_run_time", "maintenance_filter_mesh_duration", "robot", true},
	{"sensors", "sensor_run_time", "maintenance_sensor_duration", "robot", true},
	{"mop", "mop_run_time", "maintenance_mop_duration", "robot", true},
	{"swivel wheel", "swivel_wheel_run_time", "maintenance_swivel_wheel_duration", "robot", true},
	{"dust bag", "dustbag_run_time", "maintenance_dustbag_duration", "station", true},
	{"station filter tray", "sta_filter_tray_run_time", "maintenance_sta_filter_tray_duration", "station", true},
	{"dirty water filter", "dirty_water_reservoir_filter_run_time", "maintenance_dirty_water_reservoir_filter_duration", "station", true},
	{"dirty water basin", "rob_dirty_water_basin_run_time", "maintenance_dirty_water_reservoir_duration", "robot", false},
	{"scraper", "scrape_run_time", "maintenance_mid_drum_mop_wiper_duration", "robot", false},
	{"clean tray", "", "maintenance_sta_clean_tray_duration", "station", false},
	{"fabric brush", "fabricbrush_run_time", "", "station", false},
	{"dirty water filter (2)", "dirty_waterfilter_run_time", "", "station", false},
}

// consumableState is what one part reports.
type consumableState struct {
	Name      string   `json:"name"`
	Where     string   `json:"where"`
	TotalMin  *int     `json:"total_minutes"`
	LeftMin   *int     `json:"left_minutes"`
	UsedMin   *int     `json:"used_minutes"`
	Percent   *int     `json:"percent_left"`
	TotalKey  string   `json:"total_key"`
	LeftKey   string   `json:"left_key"`
	Attention bool     `json:"attention"`
	Notes     []string `json:"notes,omitempty"`
}

func readConsumables() []consumableState { return readConsumablesFrom(userConfig) }

func readConsumablesFrom(path string) []consumableState {
	out := make([]consumableState, 0, len(consumables))
	for _, c := range consumables {
		st := consumableState{Name: c.Name, Where: c.Where, TotalKey: c.Total, LeftKey: c.Left}
		if c.Total != "" {
			if v, err := scanInt(path, c.Total); err == nil {
				st.TotalMin = &v
			}
		}
		if c.Left != "" {
			if v, err := scanInt(path, c.Left); err == nil {
				st.LeftMin = &v
			}
		}
		if st.TotalMin != nil && st.LeftMin != nil && !c.paired {
			st.Notes = append(st.Notes,
				"the two keys are not confirmed to be the same part, so no age is computed")
		}
		if c.paired && st.TotalMin != nil && st.LeftMin != nil && *st.TotalMin > 0 {
			used := *st.TotalMin - *st.LeftMin
			if used < 0 {
				// The pair is the other way round from what the dust bag
				// implies. Say so rather than report a negative age.
				st.Notes = append(st.Notes, "left exceeds total; the key roles may be reversed for this part")
			} else {
				st.UsedMin = &used
				pct := *st.LeftMin * 100 / *st.TotalMin
				st.Percent = &pct
				st.Attention = pct <= 10
			}
		}
		out = append(out, st)
	}
	sort.SliceStable(out, func(i, j int) bool {
		// Anything asking for attention first, then the parts that report a
		// percentage at all, then the rest in table order.
		if out[i].Attention != out[j].Attention {
			return out[i].Attention
		}
		if (out[i].Percent == nil) != (out[j].Percent == nil) {
			return out[i].Percent != nil
		}
		if out[i].Percent != nil && out[j].Percent != nil && *out[i].Percent != *out[j].Percent {
			return *out[i].Percent < *out[j].Percent
		}
		return false
	})
	return out
}

// cleaningTotals are the lifetime counters kept beside the consumables.
func cleaningTotals() map[string]any { return cleaningTotalsFrom(userConfig) }

func cleaningTotalsFrom(path string) map[string]any {
	out := map[string]any{}
	for key, label := range map[string]string{
		"user_total_clean_area":                 "area_m2",
		"user_total_clean_duration":             "duration_minutes",
		"user_total_clean_count":                "cleans",
		"last_change_dust_bag_total_clean_area": "area_m2_at_last_bag_change",
	} {
		if v, err := scanInt(path, key); err == nil {
			out[label] = v
		}
	}
	if v, err := scanInt(path, "last_clean_time"); err == nil {
		out["last_clean_unix"] = v
	}
	if a, ok := out["area_m2"].(int); ok {
		if b, ok := out["area_m2_at_last_bag_change"].(int); ok {
			out["area_m2_since_bag_change"] = a - b
		}
	}
	return out
}

func (s *server) handleConsumables(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodGet {
		writeJSON(w, http.StatusMethodNotAllowed, map[string]string{"error": "GET"})
		return
	}
	parts := readConsumables()
	if len(parts) == 0 {
		writeJSON(w, http.StatusServiceUnavailable, map[string]any{
			"ok": false, "error": fmt.Sprintf("no consumable counters in %s", userConfig),
		})
		return
	}
	writeJSON(w, http.StatusOK, map[string]any{
		"ok":         true,
		"unit":       "minutes",
		"parts":      parts,
		"totals":     cleaningTotals(),
		"read_only":  true,
		"reset_note": "resets are not wired up; ConsumableSetting exists on the bus but is never published from here",
	})
}
