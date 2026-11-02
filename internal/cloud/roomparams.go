package cloud

// Per-room clean parameters, DP 170, sent immediately before a room clean.
//
// The app does this and x10-ctl did not, which is why a global sweep_only never
// stopped the mopping: the room's own stored parameters win. See
// docs/app-control.md for the captured frames.

import (
	"fmt"
	"github.com/dhumpf/x10-ctl/internal/dp"
	"github.com/dhumpf/x10-ctl/internal/pb"
	"strconv"
)

// roomParam is one room's settings inside DP 170.
//
// Only the fields whose meaning was established from the capture are written.
// f3.f2, f3.f3 and f6 are still unknown and stay unwritten: f3.f3 read 60, 127
// and 31 across captures and moves on its own.
type RoomParam struct {
	ID         int
	CleanType  int // 0 sweep_only, 1 mop_only, 2 sweep_and_mop, 3 sweep_then_mop
	Suction    int // 0 quiet .. 4 max_plus
	Water      int // 0 low, 1 middle, 2 high
	Intensity  int // 0 medium, 1 high, 2 low - the app's order, not ours
	CleanTimes int
}

// enumSub writes a scalar enum the way this firmware reads one: an empty
// submessage for 0, because proto3 drops a zero and an absent field means no
// change. Same shape dust.go relies on for a false Switch.
func enumSub(field uint64, v int) []byte {
	var inner []byte
	if v != 0 {
		inner = pb.FieldVarint(1, uint64(v))
	}
	return pb.FieldBytes(field, inner)
}

// roomParamsDP builds the DP 170 body: map id plus one entry per room.
func roomParamsDP(mapID uint64, rooms []RoomParam, seq uint64) []byte {
	var entries []byte
	for _, r := range rooms {
		var p []byte
		p = append(p, enumSub(1, r.CleanType)...)
		p = append(p, enumSub(2, r.Suction)...)
		// Water lives one level down, in f3.f1. f3.f2 and f3.f3 are left out.
		if r.Water != 0 {
			p = append(p, pb.FieldBytes(3, pb.FieldVarint(1, uint64(r.Water)))...)
		} else {
			p = append(p, pb.FieldBytes(3, nil)...)
		}
		p = append(p, enumSub(4, r.Intensity)...)
		if r.CleanTimes > 0 {
			p = append(p, pb.FieldVarint(5, uint64(r.CleanTimes))...)
		}
		var entry []byte
		entry = append(entry, pb.FieldVarint(1, uint64(r.ID))...)
		entry = append(entry, pb.FieldBytes(2, p)...)
		// Field 3, not 1. The app wraps each room entry in f3{f1{...}} and a
		// frame in the wrong slot is accepted and ignored.
		entries = append(entries, pb.FieldBytes(3, pb.FieldBytes(1, entry))...)
	}

	var inner []byte
	inner = append(inner, pb.FieldBytes(1, nil)...)
	inner = append(inner, pb.FieldBytes(2, nil)...)
	inner = append(inner, entries...)
	inner = append(inner, pb.FieldBytes(4, nil)...)

	var body []byte
	body = append(body, pb.FieldVarint(1, 5)...)
	body = append(body, pb.FieldVarint(2, seq)...)
	body = append(body, pb.FieldVarint(3, mapID)...)
	body = append(body, pb.FieldBytes(10, inner)...)
	return body
}

// SendRoomParams pushes the parameters for a set of rooms. Errors are the
// caller's to ignore: a clean with stale parameters still cleans.
func (c *Cloud) SendRoomParams(mapID uint64, rooms []RoomParam) error {
	if c == nil || !c.Ready() {
		return fmt.Errorf("no data-point path")
	}
	body := roomParamsDP(mapID, rooms, c.NextCmdSeq())
	return c.SendDP(map[string]any{strconv.Itoa(dp.DPRoomParams): dp.DPValue(body)})
}
