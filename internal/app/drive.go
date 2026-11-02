package app

// Manual drive.
//
// The raw chassis channel is eros.proto.msg.chassis.MotionCtrl, and driving it
// is a bad idea: `chassis_pub -m` publishes at 20 Hz in an infinite loop with no
// stop-on-exit, so killing the publisher does not stop the robot. There is no
// watchdog on that layer at all.
//
// The app layer has one. proto/cloud/control.proto, recovered from the
// binaries, says it plainly:
//
//	message RemoteCtrl {
//	  enum Direction {
//	    BRAKE = 0;   // stops the previous command without leaving remote mode,
//	                 // sent on key-up
//	    FORWARD = 1; BACK = 2; LEFT = 3; RIGHT = 4;
//	  }
//	  Direction direction = 1;
//	}
//
// with the comment that sending any of FORWARD/BACK/LEFT/RIGHT *enters* remote
// mode - there is no separate mode command to send first. robotapp carries
// HandleRemoteModeTimeOutEv, so it drops out of remote mode on its own if the
// commands stop. That timeout is the reason this path is safe and the chassis
// one is not.
//
// DP 155 is the cloud face of the same thing. Watching the bus while sending it
// showed where it lands:
//
//	{"155":0}  ->  cloud.robotdev SetRemoteCtrl direction: Brake
//	           ->  eros.proto.msg.app.FunctionalCtrl, field 2, empty submessage
//
// so field 2 of FunctionalCtrl is the RemoteCtrl, and robotapp (which attaches
// that segment) is the subscriber. Publishing it directly skips the broker.

import (
	"context"
	"fmt"
	"github.com/dhumpf/x10-ctl/internal/dp"
	"github.com/dhumpf/x10-ctl/internal/pb"
	"log"
	"net/http"
	"strings"
	"time"
)

const mdsFunctionalCtrl = "eros.proto.msg.app.FunctionalCtrl"

// Directions, as named in the recovered proto.
var directions = map[string]uint64{
	"brake": 0, "stop": 0,
	"forward": 1, "back": 2, "left": 3, "right": 4,
}

// RemoteCtrl publishes one direction onto the bus.
//
//	app.FunctionalCtrl { Header header = 1; RemoteCtrl remote = 2 }
//
// BRAKE is zero, and proto3 omits a zero varint, so it really does go out as an
// empty submessage. That is not a shortcut - it is what cloud_cli puts on the
// wire for {"155":0}.
func (l *Local) RemoteCtrl(ctx context.Context, dir uint64) error {
	var body []byte
	if dir != 0 {
		body = pb.FieldVarint(1, dir)
	}
	return l.mdsPublish(ctx, mdsFunctionalCtrl, pb.FieldBytes(2, body))
}

// handleMove drives the robot for as long as something keeps asking.
//
// Bus first, DP 155 second - the same order as every other command here, for
// the same reason: the broker is the thing being got rid of.
//
// Remote mode is 遥控清扫 - remote *clean*, not remote drive. Entering it drops
// the mops and spins the fan, which is fine on carpet and wrong on a wooden
// floor with a robot that is only being moved for debugging. Ending the task is
// what retracts them, so the caller asks for that with ?park=1 once it is done
// driving - see brakeAndPark for why that is not on every key-up.
func (s *server) handleMove(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPost {
		writeJSON(w, http.StatusMethodNotAllowed, map[string]string{"error": "POST only"})
		return
	}
	name := strings.ToLower(strings.TrimSpace(r.URL.Query().Get("dir")))
	dir, ok := directions[name]
	if !ok {
		writeJSON(w, http.StatusBadRequest, map[string]any{
			"ok": false, "error": "dir must be one of forward, back, left, right, brake",
		})
		return
	}

	ctx, cancel := context.WithTimeout(r.Context(), 20*time.Second)
	defer cancel()

	via, err := s.remoteCtrl(ctx, dir)
	if err != nil {
		writeJSON(w, http.StatusOK, map[string]any{"ok": false, "error": err.Error()})
		return
	}
	// Parking is a separate request. BRAKE deliberately stays in remote mode -
	// that is the whole point of it - because leaving costs a re-entry on the
	// next press, which reads as the robot stopping by itself mid-drive.
	parked := false
	if r.URL.Query().Get("park") == "1" {
		parked = s.brakeAndPark(ctx)
	}
	writeJSON(w, http.StatusOK, map[string]any{
		"ok": true, "dir": name, "bus": via, "parked": parked,
	})
}

// brakeAndPark ends remote mode and lifts the mops.
//
// BRAKE alone stops the wheels but explicitly does *not* leave remote mode -
// the proto says so - so the mops stay down on the floor, turning. That is
// correct between two presses and wrong once the user has finished, so this is
// driven by an idle timer rather than by every key-up: re-entering remote mode
// costs about two seconds, and paying it on every button release makes the
// robot look like it is stopping on its own.
//
// STOP_RC_CLEAN followed by STOP_TASK is what actually retracts the mops, and
// the chassis confirms it in the log:
//
//	MD_MOP:[0]RUNNING_UP, up success!
//	mop_r: limit_state = 1, limit_value = 4091, position = 10
//
// Best effort: the brake has already gone out and is the part that matters, so
// a failure here is logged rather than returned.
func (s *server) brakeAndPark(ctx context.Context) bool {
	ok := true
	for _, method := range []uint64{dp.MethodStopRCClean, dp.MethodStop} {
		seq := s.cloud.NextCmdSeq()
		if s.local.Available() && !s.noLocalCtl {
			if err := s.local.ModeCtrlParam(ctx, method, seq, nil); err == nil {
				continue
			}
		}
		if !s.cloud.Ready() {
			ok = false
			continue
		}
		if err := s.cloud.ModeCtrlParam(method, seq, nil); err != nil {
			log.Printf("brake: park via method %d: %v", method, err)
			ok = false
		}
	}
	return ok
}

func (s *server) remoteCtrl(ctx context.Context, dir uint64) (string, error) {
	if s.local.Available() && !s.noLocalCtl {
		if err := s.local.RemoteCtrl(ctx, dir); err == nil {
			return "local", nil
		}
		log.Printf("remotectrl %d: local failed, falling back", dir)
	}
	if s.cloud.Ready() {
		return "mqtt", s.cloud.SendDP(map[string]any{"155": dir})
	}
	return "", fmt.Errorf("no channel available")
}
