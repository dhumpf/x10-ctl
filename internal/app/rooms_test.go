package app

import (
	"github.com/dhumpf/x10-ctl/internal/pb"
	"testing"
)

// The order rooms are given in is the order they are cleaned in, carried as an
// explicit one-based Room.order rather than implied by position. The map page
// used to collect ticks with querySelectorAll(':checked'), which returns DOM
// order, so a user picking 7 then 1 got them sent as 1 then 7 and the encoding
// dutifully numbered them the wrong way round. This pins the encoder half.
func TestRoomsParamKeepsGivenOrder(t *testing.T) {
	body := roomsParam([]int{7, 1, 5}, 42, 1)

	type room struct{ id, order uint64 }
	var got []room
	for _, f := range pb.ParseFields(body) {
		if f.Num != 1 || f.Wire != 2 {
			continue
		}
		var r room
		for _, g := range pb.ParseFields(f.Data) {
			switch g.Num {
			case 1:
				r.id = g.Val
			case 2:
				r.order = g.Val
			}
		}
		got = append(got, r)
	}

	want := []room{{7, 1}, {1, 2}, {5, 3}}
	if len(got) != len(want) {
		t.Fatalf("got %d rooms, want %d: %v", len(got), len(want), got)
	}
	for i := range want {
		if got[i] != want[i] {
			t.Errorf("room %d: got id=%d order=%d, want id=%d order=%d",
				i, got[i].id, got[i].order, want[i].id, want[i].order)
		}
	}
}
