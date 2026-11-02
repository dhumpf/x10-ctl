package mqtt

// The transport seam between the cloud protocol (cloud.go) and the two ways
// x10-ctl can reach the robot's MQTT channel:
//
//	internal  the robot dials us. x10-ctl is broker and cloud at once, and the
//	          robot is configured to reach it locally. Nothing else has to run.
//	external  a broker runs elsewhere (broker.example.lan). Both the robot and x10-ctl
//	          are clients on it.
//
// cloud.go never learns which one it got.

import "time"

// Bus carries MQTT messages in whichever direction the mode implies.
type Bus interface {
	// Publish sends one message towards the robot.
	Publish(topic string, payload []byte) error
	// Run services the bus until stop closes. It logs and retries rather than
	// returning: the robot may be asleep, and it reconnects on its own.
	Run(stop <-chan struct{})
	// Status is what /api/mqtt renders.
	Status() BusStatus
}

type BusStatus struct {
	Mode      string `json:"mode"`
	Endpoint  string `json:"endpoint"`
	Connected bool   `json:"connected"`
	ClientID  string `json:"client_id,omitempty"`
	// pointer so an unset time is absent rather than "0001-01-01"
	Since     *time.Time `json:"since,omitempty"`
	Messages  uint64     `json:"messages"`
	LastError string     `json:"last_error,omitempty"`
}

// stamp is the nil-safe form of &t for the Since field.
func stamp(t time.Time) *time.Time {
	if t.IsZero() {
		return nil
	}
	return &t
}

// msgHandler receives every message the bus sees, robot-originated or not.
// Filtering our own traffic back out is cloud.go's job, because only it knows
// what our envelopes look like.
type MsgHandler func(topic string, payload []byte)
