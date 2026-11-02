package app

// Where things are on the map.
//
// The map is a plain occupancy PNG and the robot's world coordinates are
// metres, so putting anything on it needs the frame from house_meta_info.yaml:
//
//	resolution 0.05 m/px   width 266   height 336   origin [-8.925, -13.425]
//
// origin is the world coordinate of the map's first row, and - measured against
// the robot, not assumed - image y runs the SAME way as world y here:
//
//	px = (wx - origin_x) / resolution
//	py = (wy - origin_y) / resolution
//
// The ROS convention would invert that second line, and the dock marker drew
// vertically mirrored when it did. This is the second time on this robot that
// an obvious convention has not held; the map PNG is stored the other way up.
//
// dock_pose.yaml gives a known point to check that against: the dock sits at
// world (-0.3599, -0.0406), which lands at pixel (171, 268). If the dock marker
// draws on the dock, the transform is right - no robot movement needed to test
// it, which is how the vertical flip above was caught.
//
// The live path comes from the 1 MB buffer robotapp shares with cloud_cli,
// keyed 0x92d203d6. It is not an mds ring; it is a plain struct:
//
//	0x40 uint32  buffer size, 0x100000
//	0x4c uint32  map id, 77 here - matches the map directory
//	0x54 uint32  point count
//	0x58         count * { uint16 x, uint16 y, uint16 flag } in map pixels
//
// The count was confirmed by finding where the data stops: 376 points, 2256
// bytes, ending exactly where the buffer goes to zero.
//
// The path decode is NOT confirmed. The leading records read as sensible map
// pixels - (119,107), (118,107), (119,108), stepping by one with a constant
// flag - but 364 of 376 fall outside the image, so the buffer is not a flat
// array of pixel tuples. Either the count at 0x54 covers more than the path, or
// the records change shape partway through. Only in-bounds points are served,
// which stops the overlay drawing nonsense and is not the same as understanding
// the format.
//
// The dock is a different matter: its position comes from dock_pose.yaml and
// the transform is arithmetic, so the marker is trustworthy and is what tests
// the frame.
//
// A real live pose is DynamicData.cur_pose in the app's map stream, still not
// located.

import (
	"bufio"
	"context"
	"encoding/json"
	"fmt"
	"log"
	"math"
	"net/http"
	"os/exec"
	"path/filepath"
	"strconv"
	"strings"
	"time"
)

const pathBufferKey = 0x92d203d6

// MapFrame is the map's world-to-pixel transform.
type MapFrame struct {
	Resolution float64 `json:"resolution"`
	Width      int     `json:"width"`
	Height     int     `json:"height"`
	OriginX    float64 `json:"origin_x"`
	OriginY    float64 `json:"origin_y"`
}

// ToPixel converts world metres to image pixels.
func (f *MapFrame) ToPixel(wx, wy float64) (float64, float64) {
	if f.Resolution == 0 {
		return 0, 0
	}
	return (wx - f.OriginX) / f.Resolution, (wy - f.OriginY) / f.Resolution
}

func parseFloatKey(path, key string) (float64, bool) {
	v, ok := scanKey(path, key)
	if !ok {
		return 0, false
	}
	f, err := strconv.ParseFloat(strings.TrimSpace(v), 64)
	return f, err == nil
}

// readMapFrame reads house_meta_info.yaml.
func readMapFrame() (*MapFrame, error) {
	meta, err := readMapMeta()
	if err != nil {
		return nil, err
	}
	path := filepath.Join(meta.Dir, "house_meta_info.yaml")
	f := &MapFrame{}
	res, ok := parseFloatKey(path, "resolution")
	if !ok {
		return nil, fmt.Errorf("no resolution in %s", path)
	}
	f.Resolution = res
	if w, err := scanInt(path, "width"); err == nil {
		f.Width = w
	}
	if h, err := scanInt(path, "height"); err == nil {
		f.Height = h
	}
	// origin is a flow sequence: origin: [ -8.925, -13.425 ]
	if v, ok := scanKey(path, "origin"); ok {
		s := strings.Trim(strings.TrimSpace(v), "[]")
		parts := strings.Split(s, ",")
		if len(parts) == 2 {
			f.OriginX, _ = strconv.ParseFloat(strings.TrimSpace(parts[0]), 64)
			f.OriginY, _ = strconv.ParseFloat(strings.TrimSpace(parts[1]), 64)
		}
	}
	if f.Width == 0 || f.Height == 0 {
		return nil, fmt.Errorf("no map size in %s", path)
	}
	return f, nil
}

// DockPose is the dock in world coordinates and in map pixels.
type DockPose struct {
	X   float64 `json:"x"`
	Y   float64 `json:"y"`
	Phi float64 `json:"phi"`
	PX  float64 `json:"px"`
	PY  float64 `json:"py"`
}

func readDockPose(frame *MapFrame) (*DockPose, error) {
	meta, err := readMapMeta()
	if err != nil {
		return nil, err
	}
	path := filepath.Join(meta.Dir, "dock_pose.yaml")
	x, okx := parseFloatKey(path, "dock_x")
	y, oky := parseFloatKey(path, "dock_y")
	if !okx || !oky {
		return nil, fmt.Errorf("no dock pose in %s", path)
	}
	phi, _ := parseFloatKey(path, "dock_phi")
	d := &DockPose{X: x, Y: y, Phi: phi}
	d.PX, d.PY = frame.ToPixel(x, y)
	return d, nil
}

// Point is one map-pixel coordinate from the path buffer.
type Point struct {
	X    int `json:"x"`
	Y    int `json:"y"`
	Flag int `json:"flag"`
}

func (s *server) handlePose(w http.ResponseWriter, r *http.Request) {
	// Asking is what keeps the subscribers alive. Before the read, so a first
	// request after an idle period starts them even though it answers without
	// a pose itself.
	s.local.PoseDemand()
	frame, err := readMapFrame()
	if err != nil {
		writeJSON(w, http.StatusServiceUnavailable, map[string]any{"ok": false, "error": err.Error()})
		return
	}
	out := map[string]any{"ok": true, "frame": frame}
	if d, err := readDockPose(frame); err == nil {
		out["dock"] = d
	}

	if p, ok := s.local.Pose(); ok {
		// While charging the true map pose is known, so take the chance to
		// re-derive the transform. Cheap, and it keeps the estimate honest.
		//
		// The test used to be charge_state == 1, on the belief that a docked
		// robot with a full battery reports 0 and so the automatic path would
		// almost never fire. The belief was wrong and so was the test: sampled
		// on the dock at 100%, charge_state is 2 for every reading, and 0 was
		// only ever recorded off the dock at 97%. 1 is charging in progress, so
		// == 1 meant this fired only during the top-up and not once the pack was
		// full - the very case it was written to catch.
		//
		// Non-zero is the dock. Charging cannot happen anywhere else.
		// ?calibrate=1 still forces it, for a robot parked on the dock with a
		// charge state that says something unexpected.
		forced := r.URL.Query().Get("calibrate") == "1"
		if p.Frame == "odom" && (forced || s.local.Battery().ChargeState > 0) {
			if d, err := readDockPose(frame); err == nil {
				s.calibMu.Lock()
				s.calib = calibrateAtDock(p, d)
				s.calibMu.Unlock()
			}
		}
		out["pose"] = p
		// Only the map frame can be drawn. The odom frame has its own origin
		// and drifts, so placing it on the map would put the robot somewhere
		// convincing and wrong.
		est := p
		if p.Frame == "odom" {
			s.calibMu.Lock()
			k := s.calib
			s.calibMu.Unlock()
			// A calibration taken before the last re-zero describes a frame
			// that no longer exists, so it is dropped rather than served.
			if k != nil && s.local.jumpedSince(k.At) {
				out["calibration_stale"] = "odom was re-zeroed since the last dock reading"
				k = nil
			}
			if k != nil {
				est = k.apply(p)
				out["pose_estimated"] = est
				out["calibrated_at"] = k.At.Format(time.RFC3339)
				out["odom_travelled"] = s.local.odomTravel()
			}
		}
		if est.Frame == "map" || est.Frame == "map~odom" {
			px, py := frame.ToPixel(est.X, est.Y)
			out["pose_px"] = map[string]float64{"x": px, "y": py}
			out["pose_frame"] = est.Frame
		}
	}

	pts, mapID, err := readPathPoints()
	if err != nil {
		out["path_error"] = err.Error()
		writeJSON(w, http.StatusOK, out)
		return
	}
	// The buffer holds more than the path: past the trail the records stop
	// being pixel coordinates (y reads 65535, i.e. -1 signed), so 0x54 is not a
	// pure path count. Rather than guess where the boundary is, keep only what
	// lands inside the map. A point outside the image is not a position by any
	// reading of the format.
	inBounds := pts[:0]
	for _, p := range pts {
		if p.X >= 0 && p.X < frame.Width && p.Y >= 0 && p.Y < frame.Height {
			inBounds = append(inBounds, p)
		}
	}
	dropped := len(pts) - len(inBounds)
	pts = inBounds
	out["path_map_id"] = mapID
	if dropped > 0 {
		out["path_dropped"] = dropped
	}
	// Thin the path for the browser: 376 points is fine, a long clean is not.
	step := 1
	if len(pts) > 2000 {
		step = len(pts) / 2000
	}
	thinned := make([]Point, 0, len(pts)/step+1)
	for i := 0; i < len(pts); i += step {
		thinned = append(thinned, pts[i])
	}
	out["path"] = thinned
	out["path_points"] = len(pts)
	if len(pts) > 0 {
		last := pts[len(pts)-1]
		out["last"] = last
		out["last_note"] = "last path point, not a live pose"
	}
	writeJSON(w, http.StatusOK, out)
}

// --- live pose, off the akos bus
//
// Found in a diagnostics recording rather than by probing: the bags under
// /userdata/bigdata/akosbag are rosbag v2.0 files, and their connection records
// name every topic that was recording. /topic/chassis/odom carries
// node_sensor_msgs/OdometryPose, which akostopic renders as
//
//	{"header":{...,"frame_id":"odom"},"x":-7.13551,"y":6.59628,"theta":37.5093}
//
// Two frames matter and they are not the same:
//
//	/carto/slam/pose_stamped   map frame - what the map wants, but SLAM only
//	                           publishes while a task is running
//	/topic/chassis/odom        odom frame - always publishing, but its origin is
//	                           wherever the robot last reset and it drifts
//
// So the map frame is preferred and the odom frame is served labelled, rather
// than quietly drawn in the wrong place. theta accumulates past 2*pi - the
// sample above is 37.5 - so it is a running heading, not a wrapped angle.
//
// This also corrects the earlier reading that akos publishes nothing. It
// publishes plenty; the topics that read empty were either never advertised
// (/topic/chassis/laser_scan) or belonged to hardware that was powered down.

const (
	akosOdomTopic = "/topic/chassis/odom"
	akosSlamTopic = "/carto/slam/pose_stamped"
)

// RobotPose is where the robot thinks it is.
type RobotPose struct {
	X     float64 `json:"x"`
	Y     float64 `json:"y"`
	Theta float64 `json:"theta"`
	Frame string  `json:"frame"` // "map" or "odom"
	Age   float64 `json:"age"`
}

type akosPose struct {
	X     float64 `json:"x"`
	Y     float64 `json:"y"`
	Theta float64 `json:"theta"`
}

func (l *Local) Pose() (*RobotPose, bool) {
	l.mu.RLock()
	defer l.mu.RUnlock()
	if l.pose == nil {
		return nil, false
	}
	p := *l.pose
	p.Age = time.Since(l.poseSeen).Seconds()
	return &p, true
}

// poseIdle is how long a stream outlives the last request for a pose. Long
// enough that a page polling at 1 Hz never restarts one, short enough that a
// closed tab stops costing anything.
const poseIdle = 20 * time.Second

// PoseDemand records that somebody wants a pose. The streams follow this rather
// than running forever.
func (l *Local) PoseDemand() {
	if !l.Available() {
		return
	}
	l.mu.Lock()
	l.poseWant = time.Now()
	l.mu.Unlock()
}

func (l *Local) poseWanted() bool {
	l.mu.RLock()
	defer l.mu.RUnlock()
	return time.Since(l.poseWant) < poseIdle
}

// WatchPose follows both pose topics whenever a pose is being asked for.
//
// Streaming rather than polling, because akostopic attaches the 16 MB discovery
// segment on every run and a once-a-second poll would spawn a 36 MB process
// every second. But the streams are not free either: on 2026-08-06 the odom
// child had burned 88 minutes of CPU in 20 hours, and x10-ctl another 90
// parsing what it sent, for a readout nobody was looking at. So they run while
// somebody is asking and stop 20 s after the last request.
//
// The cost is that the first pose after an idle period is not there yet -
// akostopic takes about a second to attach and the topics publish on change.
// Callers already handle a missing pose, and a poll gets one on the next tick.
func (l *Local) WatchPose(stop <-chan struct{}) {
	if !l.Available() {
		return
	}
	go l.watchPoseTopic(stop, akosSlamTopic, "map")
	l.watchPoseTopic(stop, akosOdomTopic, "odom")
}

func (l *Local) watchPoseTopic(stop <-chan struct{}, topic, frame string) {
	for {
		select {
		case <-stop:
			return
		default:
		}
		if !l.poseWanted() {
			select {
			case <-stop:
				return
			case <-time.After(time.Second):
			}
			continue
		}
		// A stream that dies on the spot is a failing spawn, and retrying that
		// once a second is its own 36 MB-per-second problem. One that ran and
		// then ended did its job, or demand lapsed, and can restart promptly.
		started := time.Now()
		l.streamPose(stop, topic, frame)
		wait := time.Second
		if time.Since(started) < 2*time.Second {
			wait = 10 * time.Second
		}
		select {
		case <-stop:
			return
		case <-time.After(wait):
		}
	}
}

func (l *Local) streamPose(stop <-chan struct{}, topic, frame string) {
	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()
	// Two reasons to end a stream: the server is going down, or nobody has
	// asked for a pose in poseIdle. The scanner below blocks, so the demand
	// check cannot live in the read loop.
	go func() {
		t := time.NewTicker(2 * time.Second)
		defer t.Stop()
		for {
			select {
			case <-ctx.Done():
				return
			case <-stop:
				cancel()
				return
			case <-t.C:
				if !l.poseWanted() {
					cancel()
					return
				}
			}
		}
	}()

	cmd := exec.CommandContext(ctx, "/bin/sh", "-c",
		". "+appProfile+" >/dev/null 2>&1; export LD_LIBRARY_PATH=/app/lib:$LD_LIBRARY_PATH; exec "+
			akosTopic+" echo "+topic)
	out, err := cmd.StdoutPipe()
	if err != nil {
		return
	}
	if err := cmd.Start(); err != nil {
		return
	}
	defer func() {
		_ = cmd.Process.Kill()
		_ = cmd.Wait()
	}()

	sc := bufio.NewScanner(out)
	sc.Buffer(make([]byte, 0, 64*1024), 1<<20)
	for sc.Scan() {
		raw, ok := parseAkosJSON(sc.Text())
		if !ok {
			continue
		}
		var p akosPose
		if json.Unmarshal(raw, &p) != nil {
			continue
		}
		l.mu.Lock()
		if frame == "odom" && l.noteOdom(p.X, p.Y) {
			l.odomJumpAt = time.Now()
			log.Printf("pose: odom re-zeroed (jump to %.3f, %.3f) - calibration invalidated", p.X, p.Y)
		}
		// The map frame wins whenever it is fresh; odom only fills the gap while
		// SLAM is idle, which is most of the time the robot is docked.
		if frame == "map" || l.pose == nil || l.pose.Frame == "odom" ||
			time.Since(l.poseSeen) > 5*time.Second {
			l.pose = &RobotPose{X: p.X, Y: p.Y, Theta: p.Theta, Frame: frame}
			l.poseSeen = time.Now()
		}
		l.mu.Unlock()
	}
}

// --- odom to map, calibrated at the dock
//
// SLAM gives a map-frame pose but only while a task is running, so for the rest
// of the time - idle, or being driven by hand, which is exactly when you want to
// see the robot - there is only odom, whose origin is arbitrary and which drifts.
//
// Docking fixes that. dock_pose.yaml holds the dock's pose in the map frame and
// is rewritten every time the robot docks, so while charging the robot's true
// map pose is known. Reading odom at the same moment gives both sides of the
// transform:
//
//	map = R(dphi) * odom + t
//	dphi = dock.phi - wrap(odom.theta)
//	t    = dock_xy - R(dphi) * odom_xy
//
// Measured once while docked: odom (-7.1355, 6.5963) at 349.49 degrees against
// a dock at (0.0952, -0.0287) and 9.28 degrees - so nearly 20 degrees of
// rotation, and treating it as a translation would have been wrong.
//
// The result is an estimate and is labelled one. It is only as good as the last
// dock and it decays as odom drifts, which is why it is recomputed on every
// docking rather than stored.

type odomCalib struct {
	Cos, Sin float64
	TX, TY   float64
	At       time.Time
	// Odom reading the calibration was taken against. A reset moves odom
	// somewhere unrelated without the robot going anywhere, which makes the
	// transform silently wrong - see poseJumped.
	FromX, FromY float64
}

// odomJumpThreshold is the per-sample distance beyond which a change in odom is
// a re-zero rather than movement.
//
// Measured: driving home produced two discontinuities, 10.17 m and 8.6 m in a
// single one-second sample, both with theta moving tens of radians. The robot
// tops out well under 0.5 m/s, so anything past this is the frame moving, not
// the robot.
const odomJumpThreshold = 0.5

// wrapAngle folds an accumulating heading into (-pi, pi]. odom.theta runs on
// without wrapping - 37.5 rad after a few turns - so comparing it to anything
// needs this first.
func wrapAngle(a float64) float64 {
	a = math.Mod(a, 2*math.Pi)
	if a > math.Pi {
		a -= 2 * math.Pi
	}
	if a <= -math.Pi {
		a += 2 * math.Pi
	}
	return a
}

// calibrateAtDock derives the odom-to-map transform from a known dock pose.
func calibrateAtDock(odom *RobotPose, dock *DockPose) *odomCalib {
	dphi := dock.Phi - wrapAngle(odom.Theta)
	c, s := math.Cos(dphi), math.Sin(dphi)
	return &odomCalib{
		Cos: c, Sin: s,
		TX: dock.X - (c*odom.X - s*odom.Y),
		TY: dock.Y - (s*odom.X + c*odom.Y),
		At: time.Now(),
	}
}

func (k *odomCalib) apply(p *RobotPose) *RobotPose {
	return &RobotPose{
		X:     k.Cos*p.X - k.Sin*p.Y + k.TX,
		Y:     k.Sin*p.X + k.Cos*p.Y + k.TY,
		Theta: p.Theta + math.Atan2(k.Sin, k.Cos),
		Frame: "map~odom",
		Age:   p.Age,
	}
}

// poseJumped reports whether odom has been re-zeroed since a calibration was
// taken, which invalidates it.
//
// Odom is not continuous on this robot. Driving home produced two
// discontinuities in one trip - 10.17 m and 8.6 m inside a single second, with
// theta moving tens of radians - as SLAM relocalised and re-seeded the frame.
// Nothing announces it, and a calibration taken before one is not merely stale
// afterwards but wrong by however far the frame moved, which is metres.
//
// Detected by watching for movement no robot could have made. The tracked
// distance is cumulative because a slow sequence of legitimate steps must not
// look like a jump.
func (l *Local) odomTravel() float64 {
	l.mu.RLock()
	defer l.mu.RUnlock()
	return l.odomDist
}

// noteOdom accumulates travelled distance and reports a discontinuity.
func (l *Local) noteOdom(x, y float64) bool {
	jumped := false
	if l.haveLast {
		d := math.Hypot(x-l.lastX, y-l.lastY)
		if d > odomJumpThreshold {
			jumped = true
		} else {
			l.odomDist += d
		}
	}
	l.lastX, l.lastY, l.haveLast = x, y, true
	return jumped
}

// jumpedSince reports whether odom was re-zeroed after the given time.
func (l *Local) jumpedSince(t time.Time) bool {
	l.mu.RLock()
	defer l.mu.RUnlock()
	return !l.odomJumpAt.IsZero() && l.odomJumpAt.After(t)
}
