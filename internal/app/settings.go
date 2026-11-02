package app

// Cleaning settings: suction, water, clean type, extent, repeats.
//
// Read and write both stay off the broker. Writes go to
// eros.proto.msg.app.CleanSetting, which cloud_cli creates on demand the first
// time anything publishes it - `channel_config.yaml` lists 38 channels and the
// robot runs 72, so an unlisted name is not a missing channel, just an unused
// one. Confirmed on hardware: publishing directly moved the channel cursor with
// no `mqtt recv` line anywhere in the log.
//
//	CleanSetting       { Header header = 1; ...; CleanParamRequest req = 5 }
//	CleanParamRequest  { CleanParam clean_param = 1; CleanParam area_clean_param = 2 }
//
// BOTH parameter sets have to be written, and the proto says why:
//
//	通用房间清扫，清洁参数走 clean_param 中 area_clean_param 参数
//	general room clean: parameters come from area_clean_param
//
// Whole-home cleans read clean_param; ROOM cleans read area_clean_param. An
// earlier version wrote only field 1, so setting sweep-only and then starting a
// room clean produced a robot that washed its mops and mopped the room - the
// setting was real, it just was not the one that clean consulted. Worse, the
// task then wrote its own parameters back, so the change appeared to revert on
// its own.
//	CleanParam         { CleanType clean_type = 1; CleanCarpet clean_carpet = 2;
//	                     CleanExtent clean_extent = 3; MopMode mop_mode = 4;
//	                     Switch smart_mode_sw = 5; Fan fan = 6;
//	                     uint32 clean_times = 7 }
//
// The nesting was checked rather than assumed: setting fan to STANDARD produced
// exactly f5{f1{f6{f1:1}}} on the wire.
//
// Reads come from /userdata/user_config/user_config.yaml instead of the
// channel. The channel only carries the last change, so it says nothing about
// settings nobody has touched this boot; the file is the persisted truth and is
// rewritten by cloud_cli on every change.

import (
	"context"
	"fmt"
	"github.com/dhumpf/x10-ctl/internal/pb"
	"net/http"
	"strings"
	"time"
)

const (
	mdsCleanSetting = "eros.proto.msg.app.CleanSetting"
	userConfig      = "/userdata/user_config/user_config.yaml"
)

// Enum labels, straight out of the recovered clean_param.proto. Index is the
// wire value, so a lookup is also the validation.
var (
	suctionNames = []string{"quiet", "standard", "turbo", "max", "max_plus"}
	mopNames     = []string{"low", "middle", "high"}
	typeNames    = []string{"sweep_only", "mop_only", "sweep_and_mop", "sweep_then_mop"}
	extentNames  = []string{"normal", "narrow", "quick"}
)

// CleanSettings is the settings we expose. Pointers so a request can carry only
// the fields it wants to change: an absent submessage means "no change" to the
// robot, and modelling that as a zero value would silently rewrite everything.
type CleanSettings struct {
	CleanType  *int  `json:"clean_type"`
	Fan        *int  `json:"fan"`
	MopWater   *int  `json:"mop_water"`
	Extent     *int  `json:"clean_extent"`
	CleanTimes *int  `json:"clean_times"`
	SmartMode  *bool `json:"smart_mode"`
	// CarpetStrategy is CleanParam field 2, clean_carpet_strategy. Setting
	// carpet handling to avoidance in the vendor app on 2026-08-06 sent
	// f2 { f1: 1 } as DP 154 and moved clean_carpet_strategy to 1, while
	// corner_clean and clean_carpet_mode both stayed at 0. Setting it back
	// sent f2 {}, empty, because 0 is the proto3 default.
	//
	// Not to be confused with corner_clean, which is field 2 of MopMode, and
	// MopMode is CleanParam field 4. Two different field 2s, one nested.
	CarpetStrategy *int `json:"carpet_strategy"`
}

func nameOf(list []string, v *int) string {
	if v == nil || *v < 0 || *v >= len(list) {
		return ""
	}
	return list[*v]
}

// readSettings reports the persisted cleaning parameters.
func readSettings() (*CleanSettings, error) {
	s := &CleanSettings{}
	get := func(key string) *int {
		if v, err := scanInt(userConfig, key); err == nil {
			return &v
		}
		return nil
	}
	s.CleanType = get("clean_type")
	s.Fan = get("fan_level")
	s.MopWater = get("mop_water_level")
	s.Extent = get("clean_extent_level")
	s.CarpetStrategy = get("clean_carpet_strategy")
	if v, ok := scanKey(userConfig, "smart_mode_enable"); ok {
		b := strings.TrimSpace(v) == "true"
		s.SmartMode = &b
	}
	if s.CleanType == nil && s.Fan == nil {
		return nil, fmt.Errorf("no cleaning settings in %s", userConfig)
	}
	return s, nil
}

// cleanParam encodes a partial CleanParam. Only the fields the caller set are
// emitted, so everything else keeps its current value.
func cleanParam(s *CleanSettings) []byte {
	var p []byte
	sub := func(field uint64, v *int) {
		if v == nil {
			return
		}
		var inner []byte
		if *v != 0 {
			inner = pb.FieldVarint(1, uint64(*v))
		}
		p = append(p, pb.FieldBytes(field, inner)...)
	}
	sub(1, s.CleanType)
	sub(3, s.Extent)
	sub(2, s.CarpetStrategy) // clean_carpet_strategy
	sub(4, s.MopWater)       // MopMode.level; corner_clean is MopMode field 2, left alone
	sub(6, s.Fan)
	if s.SmartMode != nil {
		var inner []byte
		if *s.SmartMode {
			inner = pb.FieldVarint(1, 1)
		}
		p = append(p, pb.FieldBytes(5, inner)...)
	}
	if s.CleanTimes != nil && *s.CleanTimes > 0 {
		p = append(p, pb.FieldVarint(7, uint64(*s.CleanTimes))...)
	}
	return p
}

// SetCleanParams publishes a partial CleanParam onto the bus.
//
// Every write makes the robot play a voice prompt (0002.mp3). That is its own
// acknowledgement, not a fault - the vendor app triggers the same one - and it
// follows the voice volume, DP 161.
func (l *Local) SetCleanParams(ctx context.Context, s *CleanSettings) error {
	param := cleanParam(s)
	if len(param) == 0 {
		return fmt.Errorf("nothing to set")
	}
	// Same parameters into both: a caller changing "suction" means everywhere,
	// not "everywhere except room cleans".
	req := pb.FieldBytes(1, param)
	req = append(req, pb.FieldBytes(2, param)...)
	return l.mdsPublish(ctx, mdsCleanSetting, pb.FieldBytes(5, req))
}

func (s *server) handleSettings(w http.ResponseWriter, r *http.Request) {
	if r.Method == http.MethodGet {
		cur, err := readSettings()
		if err != nil {
			writeJSON(w, http.StatusServiceUnavailable, map[string]any{"ok": false, "error": err.Error()})
			return
		}
		writeJSON(w, http.StatusOK, map[string]any{
			"ok": true, "settings": cur,
			"names": map[string]any{
				"clean_type": typeNames, "fan": suctionNames,
				"mop_water": mopNames, "clean_extent": extentNames,
			},
			"labels": map[string]string{
				"clean_type": nameOf(typeNames, cur.CleanType),
				"fan":        nameOf(suctionNames, cur.Fan),
				"mop_water":  nameOf(mopNames, cur.MopWater),
			},
		})
		return
	}
	if r.Method != http.MethodPost {
		writeJSON(w, http.StatusMethodNotAllowed, map[string]string{"error": "GET or POST"})
		return
	}

	var body CleanSettings
	if err := decodeJSON(w, r, &body); err != nil {
		writeJSON(w, http.StatusBadRequest, map[string]any{"ok": false, "error": "bad json"})
		return
	}
	for _, c := range []struct {
		v    *int
		list []string
		name string
	}{
		{body.CleanType, typeNames, "clean_type"},
		{body.Fan, suctionNames, "fan"},
		{body.MopWater, mopNames, "mop_water"},
		{body.Extent, extentNames, "clean_extent"},
	} {
		if c.v != nil && (*c.v < 0 || *c.v >= len(c.list)) {
			writeJSON(w, http.StatusBadRequest, map[string]any{
				"ok": false, "error": fmt.Sprintf("%s must be 0..%d", c.name, len(c.list)-1),
			})
			return
		}
	}

	ctx, cancel := context.WithTimeout(r.Context(), 15*time.Second)
	defer cancel()

	via := "local"
	if err := s.local.SetCleanParams(ctx, &body); err != nil {
		writeJSON(w, http.StatusOK, map[string]any{"ok": false, "error": err.Error()})
		return
	}
	// cloud_cli rewrites user_config.yaml as part of handling this, so reading
	// straight back returns the value from before the change.
	select {
	case <-ctx.Done():
	case <-time.After(700 * time.Millisecond):
	}
	cur, _ := readSettings()
	writeJSON(w, http.StatusOK, map[string]any{"ok": true, "bus": via, "settings": cur})
}
