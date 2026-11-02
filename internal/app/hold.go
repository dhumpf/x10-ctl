package app

// On-demand sensor power.
//
// The lidar and the camera both cost power and wear, and both are off whenever
// the robot is parked. Leaving either latched on because a browser tab was open
// once is the wrong default; so is making the user press a button before the
// page shows anything.
//
// A hold splits the difference. Touch() switches the sensor on if it is off and
// bumps a deadline. A background sweep switches it off once nothing has touched
// it for the idle window. Polling a page keeps it alive; closing the tab lets it
// lapse on its own.
//
// Switching on is synchronous because the caller is about to read the sensor and
// a scan taken before the motor spins up is empty. Switching off is not: nobody
// is waiting on it.

import (
	"context"
	"log"
	"sync"
	"time"
)

type hold struct {
	name string
	idle time.Duration
	set  func(context.Context, bool) error

	// warmup is how long the hardware needs after the switch before a read is
	// worth taking. The lidar reports "data len not enough" for about a second
	// after power-on while the motor comes up to its 7 Hz.
	warmup time.Duration

	mu       sync.Mutex
	on       bool
	deadline time.Time
}

func newHold(name string, idle, warmup time.Duration, set func(context.Context, bool) error) *hold {
	return &hold{name: name, idle: idle, warmup: warmup, set: set}
}

// Touch switches the sensor on if needed and extends its lease. It reports
// whether it had to switch on, so a caller that cares can say "warming up"
// rather than "no data".
func (h *hold) Touch(ctx context.Context) (started bool, err error) {
	h.mu.Lock()
	already, warm := h.on, h.warmup
	h.deadline = time.Now().Add(h.idle)
	h.mu.Unlock()

	if already {
		return false, nil
	}
	if err := h.set(ctx, true); err != nil {
		return false, err
	}
	h.mu.Lock()
	h.on = true
	h.mu.Unlock()

	if warm > 0 {
		select {
		case <-ctx.Done():
		case <-time.After(warm):
		}
	}
	return true, nil
}

// Release switches the sensor off now, whatever the lease says.
func (h *hold) Release(ctx context.Context) error {
	h.mu.Lock()
	if !h.on {
		h.mu.Unlock()
		return nil
	}
	h.on = false
	h.mu.Unlock()
	return h.set(ctx, false)
}

func (h *hold) Held() bool {
	h.mu.Lock()
	defer h.mu.Unlock()
	return h.on
}

// Sweep drops the hold once its lease expires. Runs until stop closes.
func (h *hold) Sweep(stop <-chan struct{}) {
	tick := time.NewTicker(time.Second)
	defer tick.Stop()
	for {
		select {
		case <-stop:
			// Do not leave the hardware running just because we are going
			// away. This is the one path that matters for a service restart.
			ctx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
			_ = h.Release(ctx)
			cancel()
			return
		case <-tick.C:
		}
		h.mu.Lock()
		expired := h.on && time.Now().After(h.deadline)
		h.mu.Unlock()
		if !expired {
			continue
		}
		ctx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
		if err := h.Release(ctx); err != nil {
			log.Printf("hold %s: release: %v", h.name, err)
		}
		cancel()
	}
}
