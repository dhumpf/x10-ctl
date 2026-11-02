package app

import (
	"testing"
	"time"
)

// docked is the battery report this robot publishes sitting on its dock with a
// full pack: charge_state 2, not 1 and not 0.
var docked = Battery{Level: 100, ChargeState: 2, Valid: true}

// offDock is the report recorded at 97% away from the dock.
var offDock = Battery{Level: 97, ChargeState: 0, Valid: true}

func TestReconcileStatusCorrectsCleaningOnTheDock(t *testing.T) {
	got, derived := reconcileStatus("cleaning", docked, 0)
	if got != "docked" || !derived {
		t.Fatalf("got %q derived=%v, want docked/true", got, derived)
	}
}

// A state seen on the dock that is not "cleaning" is left alone. Washing and
// drying both happen docked and report states this code has no name for;
// overwriting them with "docked" would lose the only report of them there is.
func TestReconcileStatusLeavesOtherDockedStates(t *testing.T) {
	for _, st := range []string{"docked", "transient", "state 7"} {
		if got, derived := reconcileStatus(st, docked, 0); got != st || derived {
			t.Errorf("%q became %q (derived=%v)", st, got, derived)
		}
	}
}

func TestReconcileStatusFillsAnEmptyState(t *testing.T) {
	if got, derived := reconcileStatus("", docked, 0); got != "docked" || !derived {
		t.Fatalf("got %q derived=%v", got, derived)
	}
}

// Off the dock the charge state knows nothing about what the robot is doing, so
// the report stands - whatever it says.
func TestReconcileStatusPassesThroughOffDock(t *testing.T) {
	for _, st := range []string{"cleaning", "transient", "state 12"} {
		if got, derived := reconcileStatus(st, offDock, time.Hour); got != st || derived {
			t.Errorf("%q became %q (derived=%v)", st, got, derived)
		}
	}
}

// The one off-dock correction: a cached "docked" with the charge state at zero
// for longer than a charge controller's gap between top-ups.
func TestReconcileStatusStaleDockedNeedsTheGrace(t *testing.T) {
	if got, derived := reconcileStatus("docked", offDock, dockGrace/2); got != "docked" || derived {
		t.Fatalf("too early: got %q derived=%v", got, derived)
	}
	if got, derived := reconcileStatus("docked", offDock, dockGrace+time.Second); got != "off_dock" || !derived {
		t.Fatalf("after the grace: got %q derived=%v", got, derived)
	}
}

// With no battery report there is nothing to reconcile against, and inventing a
// dock state from a zero-valued struct would be worse than saying nothing.
func TestReconcileStatusNeedsAValidBattery(t *testing.T) {
	if got, derived := reconcileStatus("cleaning", Battery{}, time.Hour); got != "cleaning" || derived {
		t.Fatalf("got %q derived=%v", got, derived)
	}
}
