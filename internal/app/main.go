// x10-ctl - local web control for the eufy X10 Pro Omni.
//
// Single static binary, no cloud, no config file to hand-write: the robot
// announces itself on the LAN (discover.go), so x10-ctl auto-configures for it
// and talks to it on TCP 9668 (client.go).
//
// Build for the robot:
//
//	GOOS=linux GOARCH=arm64 CGO_ENABLED=0 go build -ldflags="-s -w" -o x10-ctl-arm64 ./cmd/x10-ctl
//
// Run anywhere on the LAN, or on the robot itself in /userdata (which survives
// an OTA, unlike /app).
package app

import (
	"context"
	"crypto/rand"
	"encoding/hex"
	"encoding/json"
	"flag"
	"fmt"
	"github.com/dhumpf/x10-ctl/internal/client"
	"github.com/dhumpf/x10-ctl/internal/cloud"
	"github.com/dhumpf/x10-ctl/internal/dp"
	"github.com/dhumpf/x10-ctl/internal/mqtt"
	"log"
	"net"
	"net/http"
	"os"
	"os/signal"
	"strconv"
	"strings"
	"sync"
	"sync/atomic"
	"syscall"
	"time"
)

// Buttons rendered in the UI. move marks the ones that physically drive the
// robot, so the page can warn before someone taps one on a phone.
//
// bus is the channel that actually carries it on this device. Movement and
// beep now ride the internal eros::mds bus, so they need no broker and no
// cloud. Only the station actions still have to go out as MQTT data points.
// Untested marks an action never fired on this hardware. The dock actions come
// from a proto shared across models, and detergent_sw proves a field can exist
// there with no mechanism behind it.
var Buttons = []struct {
	Cmd      string `json:"cmd"`
	Label    string `json:"label"`
	Move     bool   `json:"move"`
	Bus      string `json:"bus"`
	Untested bool   `json:"untested"`
}{
	{"gohome", "Dock", true, "bus", false},
	{"gowash", "Wash mop", true, "bus", false},
	{"resume", "Resume", true, "bus", false},
	{"pause", "Pause", false, "bus", false},
	{"stop", "Stop", false, "bus", false},
	{"autoclean", "Auto clean", true, "bus", false},
	{"fastmap", "Fast map", true, "bus", false},
	{"rcclean", "Manual mode", true, "bus", false},
	{"stoprcclean", "Exit manual", false, "bus", false},
	{"beep", "Find robot", false, "bus", false},
	{"beep-off", "Stop beeping", false, "bus", false},
	{"dry", "Dry mop", false, "bus", false},
	{"dust", "Empty dustbin", false, "bus", false},
	{"wash", "Station wash", false, "mqtt", true},
	{"cut-hair", "Cut hair", false, "mqtt", true},
	{"descale", "Descale", false, "mqtt", true},
	{"self-maintain", "Self maintain", false, "mqtt", true},
}

// dpCommands are the buttons that do not go over 9668. Keyed the same way as
// Methods in client.go so handleCommand can route on the name alone.
//
// beep prefers the on-device path: audio_cli reaches dev.audio directly, with
// no broker, no cloud and no data point in between. The rest have no local
// equivalent yet and still go out as MQTT data points.
var dpCommands = map[string]func(*server, context.Context) (string, error){
	"beep":          func(s *server, ctx context.Context) (string, error) { return s.beep(ctx, true) },
	"beep-off":      func(s *server, ctx context.Context) (string, error) { return s.beep(ctx, false) },
	"dry":           func(s *server, ctx context.Context) (string, error) { return s.station(ctx, "dry") },
	"dust":          func(s *server, ctx context.Context) (string, error) { return s.station(ctx, "dust") },
	"wash":          func(s *server, ctx context.Context) (string, error) { return s.station(ctx, "wash") },
	"descale":       func(s *server, ctx context.Context) (string, error) { return s.station(ctx, "descale") },
	"cut-hair":      func(s *server, ctx context.Context) (string, error) { return s.station(ctx, "cut-hair") },
	"self-maintain": func(s *server, ctx context.Context) (string, error) { return s.station(ctx, "self-maintain") },
}

// beep goes local when we are on the robot, and falls back to the data point
// when we are not. Both end at the same place - dev.audio - but the local one
// needs nothing else to be running.
func (s *server) beep(ctx context.Context, on bool) (string, error) {
	if s.local.Available() {
		if err := s.local.Beep(ctx, on); err == nil {
			return "local", nil
		} else if !s.cloud.Ready() {
			return "local", err
		} else {
			log.Printf("beep: local path failed (%v), falling back to MQTT", err)
		}
	}
	return "mqtt", s.cloud.Beep(on)
}

// runMethod drives one ModeCtrl method down the best channel and reports which
// carried it. Shared by handleCommand and the HA bridge; 9668 stays in the
// caller, being broken for movement.
// dataPointOnly are the methods the mds bus accepts and does not act on. The
// publish succeeds either way, so the local path must be skipped, not tried.
var dataPointOnly = map[string]bool{"pause": true, "resume": true}

func (s *server) runMethod(ctx context.Context, name string, method uint64) (string, error) {
	if dataPointOnly[name] {
		if !s.cloud.Ready() {
			return "", fmt.Errorf("%s needs the data-point path, which needs a broker", name)
		}
		return "mqtt", s.cloud.ModeCtrl(method)
	}
	if s.local.Available() && !s.noLocalCtl {
		if err := s.local.ModeCtrl(ctx, method, s.nextSeq()); err == nil {
			return "local", nil
		} else {
			log.Printf("%s: local mds path failed (%v), falling back", name, err)
		}
	}
	if s.cloud.Ready() {
		if err := s.cloud.ModeCtrl(method); err == nil {
			return "mqtt", nil
		} else {
			log.Printf("%s: mqtt path failed (%v), trying 9668", name, err)
		}
	}
	return "", fmt.Errorf("no channel available for %s", name)
}

// modeCtrlParam sends a parameterised ModeCtrlRequest down the best channel.
func (s *server) modeCtrlParam(r *http.Request, method uint64, param []byte) (string, error) {
	ctx, cancel := context.WithTimeout(r.Context(), 15*time.Second)
	defer cancel()
	return s.modeCtrlParamCtx(ctx, method, param)
}

// modeCtrlParamCtx is the same without an HTTP request, for the HA bridge.
func (s *server) modeCtrlParamCtx(ctx context.Context, method uint64, param []byte) (string, error) {
	seq := s.nextSeq()
	// Both the HTTP handler and the HA bridge come through here. Logging the
	// bytes is how you tell "same command, different result" from "not the same
	// command at all".
	log.Printf("modectrl: method %d seq %d param %x", method, seq, param)
	if s.local.Available() && !s.noLocalCtl {
		if err := s.local.ModeCtrlParam(ctx, method, seq, param); err == nil {
			return "local", nil
		}
		log.Printf("modectrl %d: local failed, falling back", method)
	}
	if s.cloud.Ready() {
		return "mqtt", s.cloud.ModeCtrlParam(method, seq, param)
	}
	return "", fmt.Errorf("no channel available")
}

type server struct {
	client *client.Client
	disc   *client.Discoverer
	cloud  *cloud.Cloud
	local  *Local
	// noLocalCtl forces movement back onto the older transports. The mds path
	// is the newest and least proven of the three; this is the way out.
	noLocalCtl bool
	start      time.Time

	// Sensors the robot leaves powered down while parked. Held only while a
	// page is actually asking for them; see hold.go.
	laser  *hold
	camera *hold

	lightMu sync.Mutex
	light   int // last commanded LED level, so the UI can show it back

	// Command sequence. Used to live on Cloud, which made every bus command
	// depend on a broker client existing just to get a number.
	seq atomic.Uint64

	// Clean type in force before hand-driving forced SWEEP_ONLY, so leaving
	// manual mode does not reconfigure the next scheduled clean.
	manualMu   sync.Mutex
	manualPrev *int

	// odom-to-map transform, re-derived every time the robot is seen docked.
	calibMu sync.Mutex
	calib   *odomCalib

	scanMu  sync.Mutex
	scanned []AP
	scanAt  time.Time
}

// nextSeq numbers a command. Seeded away from zero because the robot's own
// replies echo the sequence, and a run starting at 1 is indistinguishable from
// the app's traffic when reading a log.
func (s *server) nextSeq() uint64 { return 100 + s.seq.Add(1) }

func (s *server) setLight(level int) {
	s.lightMu.Lock()
	s.light = level
	s.lightMu.Unlock()
}

func (s *server) lightLevel() int {
	s.lightMu.Lock()
	defer s.lightMu.Unlock()
	return s.light
}

// Run configures x10-ctl from the command line and serves until signalled.
func Run() {
	var (
		addr    = flag.String("addr", ":8080", "listen address for the web UI")
		host    = flag.String("host", "", "robot IP (default: learned from the UDP 9667 broadcast)")
		port    = flag.Int("port", 9668, "robot control port")
		sn      = flag.String("sn", "", "device serial (default: learned from broadcast)")
		uid     = flag.String("user-id", "", "account id (default: learned from broadcast)")
		noUser  = flag.Bool("no-user", false, "omit user_id from SocketVerify")
		cache   = flag.String("cache", "x10-ctl-discovery.json", "where to cache the discovered identity")
		timeout = flag.Duration("timeout", 6*time.Second, "per-request robot timeout")

		// MQTT. Beep, battery and drying are data points, and SocketTransData
		// has no DP-number field, so none of them are reachable over 9668 -
		// see dp.go. Both paths exist because they answer different questions:
		// internal needs nothing else running, external puts the robot and other
		// clients on one broker.
		// Off by default. Everything the UI does now has a bus path, and the
		// broker was only ever a way in from outside the robot. Wash, descale
		// and cut-hair are the exception - no bus encoding yet - and they say so
		// rather than failing quietly.
		mqttMode   = flag.String("mqtt", "off", "MQTT path: off, internal (embedded broker), external (join a broker)")
		mqttListen = flag.String("mqtt-listen", ":8883", "internal: listen address for the embedded broker")
		mqttCert   = flag.String("mqtt-cert", "x10-ctl-broker.crt", "internal: TLS certificate, self-signed and written if absent")
		mqttKey    = flag.String("mqtt-key", "x10-ctl-broker.key", "internal: TLS key")
		mqttBroker = flag.String("mqtt-broker", "", "external: broker host:port, e.g. broker.example.lan:8883")
		mqttCA     = flag.String("mqtt-ca", "", "external: CA bundle for the broker (default: system roots)")
		mqttPlain  = flag.Bool("mqtt-plain", false, "speak MQTT without TLS - testing only, the robot will not")
		mqttSkip   = flag.Bool("mqtt-insecure", false, "external: skip broker certificate verification")
		mqttApp    = flag.String("mqtt-app", "eufy_home", "topic app-name segment; the robot drops it after a reset and the live topic wins")
		mqttID     = flag.String("mqtt-client-id", "", "external: client id (default: x10-ctl-<random>)")
		mqttPoll   = flag.Duration("mqtt-poll", 60*time.Second, "DP 153 clock poll interval, as the app does; 0 disables")
		apFallback = flag.String("ap-fallback", "", "SSID for an access point to raise when wlan0 has no address; empty disables it")
		apPass     = flag.String("ap-pass", "", "password for that access point, empty for open")
		apGrace    = flag.Duration("ap-grace", 90*time.Second, "how long wlan0 may be down before the access point goes up")
		noLocalCtl = flag.Bool("no-local-ctrl", false, "do not route movement through the on-device mds bus, even when available")
		haOn       = flag.Bool("ha", false, "publish Home Assistant autodiscovery to a broker")
		haBroker   = flag.String("ha-broker", "", "ha: broker host:port (default: --mqtt-broker)")
		haCA       = flag.String("ha-ca", "", "ha: CA bundle for that broker (default: --mqtt-ca)")
		haSkip     = flag.Bool("ha-insecure", false, "ha: skip broker certificate verification")
		haPlain    = flag.Bool("ha-plain", false, "ha: speak MQTT without TLS")
		haUser     = flag.String("ha-user", "", "ha: broker username, if it wants one")
		haPass     = flag.String("ha-pass", "", "ha: broker password")
		haEvery    = flag.Duration("ha-interval", 10*time.Second, "ha: how often to look for changed state")

		// The robot's own cloud, served on the robot. Off unless asked for: it
		// wants :80 and only means anything when the robot is configured to
		// reach its cloud here. See cloudapi.go.
		apiAddr   = flag.String("api-addr", "", "serve the robot's cloud API here, e.g. :80; empty disables it")
		apiDir    = flag.String("api-dir", "cloudapi", "api: directory holding the captured payloads")
		apiBroker = flag.String("api-broker", "127.0.0.1:8883", "api: broker host:port to advertise in get_mqtt_info")
		apiCA     = flag.String("api-ca", "", "api: certificate handed over as the broker CA (default: --mqtt-cert)")
	)
	flag.Parse()

	// robotapp arms every file in the schedule dir, backup copies included.
	if moved, err := quarantineStraySchedules(); err != nil {
		log.Printf("schedule: quarantine failed: %v", err)
	} else if len(moved) > 0 {
		log.Printf("schedule: moved %d stray file(s) to %s: %s - armed until the next reboot",
			len(moved), scheduleQuarantine, strings.Join(moved, ", "))
	}

	c := client.NewClient(*host, *port, *sn, *uid, *timeout)
	d := client.NewDiscoverer(*cache)
	cloud := cloud.NewCloud(*sn, *mqttApp)
	local := NewLocal()

	stop := make(chan struct{})
	// Explicit flags win over discovery; otherwise the listener fills them in.
	if *sn == "" {
		go d.Run(c, stop)
		if last := d.Last(); last != nil {
			cloud.SetIdentity(last.DeviceSN)
		}
	} else {
		log.Printf("identity from flags: %s at %s", *sn, *host)
	}
	if *noUser {
		h, _, s, _ := c.Identity()
		c.SetIdentity(h, s, "")
		log.Print("user_id omitted from SocketVerify (--no-user)")
	}

	if bus, err := buildBus(*mqttMode, cloud, busOpts{
		listen: *mqttListen, cert: *mqttCert, key: *mqttKey,
		broker: *mqttBroker, ca: *mqttCA, plain: *mqttPlain,
		insecure: *mqttSkip, clientID: *mqttID,
	}); err != nil {
		log.Fatalf("mqtt: %v", err)
	} else if bus != nil {
		cloud.SetBus(bus)
		go bus.Run(stop)
		// Discovery learns the serial from UDP 9667; MQTT topics need the same
		// value, so keep them in step rather than making the user pass --sn.
		go followDiscovery(d, cloud, stop)
		if *mqttPoll > 0 {
			// Without this the robot stays silent: it dumps state once at
			// session setup, long before we join the broker. See PollStatus.
			log.Printf("mqtt: polling DP %d every %s", dp.DPWorkStatus, *mqttPoll)
			go cloud.Poll(*mqttPoll, stop)
		}
	} else {
		log.Print("mqtt: disabled (--mqtt=off); beep, battery and drying are unavailable")
	}

	if local.Available() {
		// The bus publishes BatteryInfo every 2 s regardless of session, which
		// is the one thing the data-point channel structurally cannot do.
		log.Print("local: on-device tools present, using the chassis bus for battery")
		// Schedules are wall-clock, so a robot that boots thinking it is 1970
		// quietly never cleans. Nothing else on the device sets the clock.
		local.RestoreClock()
		go local.WatchBattery(stop)
		// Work status too: same reason, and the same accumulate-and-remember
		// shape, because the reports are partial.
		go local.WatchStatus(stop)
		go local.WatchSensors(stop)
		go local.WatchPose(stop)
	} else {
		log.Print("local: not on the robot, telemetry and beep stay on the MQTT path")
	}

	s := &server{client: c, disc: d, cloud: cloud, local: local, noLocalCtl: *noLocalCtl, start: time.Now()}

	// The lidar needs about a second after power-on before it reports a full
	// sweep; the camera is streaming within one frame. Both leases are long
	// enough to survive a page that polls once a second and a user who pauses
	// to look at something.
	s.laser = newHold("laser", 20*time.Second, 1200*time.Millisecond, local.LaserPower)
	// Dropping the stream drops the light with it: an LED left on by a closed
	// tab is worse than a stale frame, and light without a camera is pointless.
	s.camera = newHold("camera", 20*time.Second, 0, func(ctx context.Context, on bool) error {
		if !on && s.lightLevel() > 0 {
			if err := local.CameraLight(ctx, 0); err != nil {
				log.Printf("camera: light off: %v", err)
			}
			s.setLight(0)
		}
		return local.CameraStream(ctx, on)
	})
	if local.Available() {
		go s.laser.Sweep(stop)
		go s.camera.Sweep(stop)
	}

	if *haOn {
		startHA(&haConfig{
			srv: s, sn: *sn, disc: d,
			broker: pick(*haBroker, *mqttBroker), ca: pick(*haCA, *mqttCA),
			insecure: *haSkip, plain: *haPlain,
			user: *haUser, pass: *haPass, every: *haEvery,
		}, stop)
	}

	mux := http.NewServeMux()
	mux.HandleFunc("/", s.handleIndex)
	mux.HandleFunc("/api/status", s.handleStatus)
	mux.HandleFunc("/api/command", s.handleCommand)
	mux.HandleFunc("/api/mqtt", s.handleMQTT)
	mux.HandleFunc("/api/dp", s.handleDP)
	mux.HandleFunc("/api/camera.jpg", s.handleCamera)
	mux.HandleFunc("/api/camera", s.handleCameraMeta)
	mux.HandleFunc("/api/map.png", s.handleMap)
	mux.HandleFunc("/api/map", s.handleValetudoMap)
	mux.HandleFunc("/api/schedules", s.handleSchedules)
	mux.HandleFunc("/api/lidar", s.handleLidar)
	mux.HandleFunc("/api/rooms", s.handleRooms)
	mux.HandleFunc("/api/roomclean", s.handleRoomClean)
	mux.HandleFunc("/api/zoneclean", s.handleZoneClean)
	mux.HandleFunc("/api/spotclean", s.handleSpotClean)
	mux.HandleFunc("/api/move", s.handleMove)
	mux.HandleFunc("/api/camera/light", s.handleCameraLight)
	mux.HandleFunc("/api/station", s.handleStation)
	mux.HandleFunc("/api/wash", s.handleWash)
	mux.HandleFunc("/api/dry", s.handleDry)
	mux.HandleFunc("/api/peakcharge", s.handlePeakCharge)
	mux.HandleFunc("/api/wifi", s.handleWiFi)
	mux.HandleFunc("/api/time", s.handleTime)
	mux.HandleFunc("/api/pose", s.handlePose)
	mux.HandleFunc("/api/roomsettings", s.handleRoomSettings)
	mux.HandleFunc("/api/roomrename", s.handleRoomRename)
	mux.HandleFunc("/api/roomsbatch", s.handleRoomsBatch)
	mux.HandleFunc("/api/roomsmerge", s.handleRoomsMerge)
	mux.HandleFunc("/api/roomdivide", s.handleRoomDivide)
	mux.HandleFunc("/api/maprevert", s.handleMapRevert)
	mux.HandleFunc("/api/zones", s.handleZones)
	mux.HandleFunc("/api/sensors", s.handleSensors)
	mux.HandleFunc("/api/manual", s.handleManualMode)
	mux.HandleFunc("/api/mds", s.handleMDS)
	mux.HandleFunc("/api/settings", s.handleSettings)
	mux.HandleFunc("/api/undisturbed", s.handleUndisturbed)
	mux.HandleFunc("/api/volume", s.handleVolume)
	mux.HandleFunc("/api/dust", s.handleDust)
	mux.HandleFunc("/api/unisettings", s.handleUniSettings)
	mux.HandleFunc("/api/consumables", s.handleConsumables)
	mux.HandleFunc("/api/schedule", s.handleSchedule)
	mux.HandleFunc("/api/schedulerooms", s.handleScheduleRooms)

	if *apFallback != "" {
		go s.WatchWiFi(stop, *apFallback, *apPass, *apGrace)
	}

	if *apiAddr != "" {
		startCloudAPI(*apiAddr, *apiDir, *apiBroker, pick(*apiCA, *mqttCert), stop)
	}

	srv := &http.Server{
		Addr:              *addr,
		Handler:           mux,
		ReadHeaderTimeout: 5 * time.Second,
	}

	go func() {
		log.Printf("x10-ctl -> http://%s", *addr)
		if err := srv.ListenAndServe(); err != nil && err != http.ErrServerClosed {
			log.Fatalf("listen: %v", err)
		}
	}()

	sig := make(chan os.Signal, 1)
	signal.Notify(sig, os.Interrupt, syscall.SIGTERM)
	<-sig
	close(stop)
	log.Print("shutting down")
	_ = srv.Close()
}

type busOpts struct {
	listen, cert, key    string
	broker, ca, clientID string
	plain, insecure      bool
}

// buildBus turns --mqtt into a transport. A nil Bus with a nil error means the
// user asked for off.
func buildBus(mode string, cloud *cloud.Cloud, o busOpts) (mqtt.Bus, error) {
	switch mode {
	case "off", "none", "":
		return nil, nil

	case "internal":
		return mqtt.NewBrokerBus(o.listen, o.cert, o.key, o.plain, cloud.Handle)

	case "external":
		if o.broker == "" {
			return nil, fmt.Errorf("--mqtt=external needs --mqtt-broker host:port")
		}
		id := o.clientID
		if id == "" {
			b := make([]byte, 8)
			_, _ = rand.Read(b)
			id = "x10-ctl-" + hex.EncodeToString(b)
		}
		return mqtt.NewClientBus(o.broker, id, o.ca, o.insecure, o.plain, cloud.Handle), nil
	}
	return nil, fmt.Errorf("unknown --mqtt mode %q (internal, external, off)", mode)
}

// followDiscovery keeps the MQTT topics pointed at whatever serial the UDP
// broadcast reports, so neither path needs hand-written config.
func followDiscovery(d *client.Discoverer, cloud *cloud.Cloud, stop <-chan struct{}) {
	t := time.NewTicker(5 * time.Second)
	defer t.Stop()
	for {
		select {
		case <-stop:
			return
		case <-t.C:
			if last := d.Last(); last != nil {
				cloud.SetIdentity(last.DeviceSN)
			}
		}
	}
}

// startCloudAPI serves the robot's own cloud API on its own listener.
func startCloudAPI(addr, dir, broker, ca string, stop <-chan struct{}) {
	host, portStr, err := net.SplitHostPort(broker)
	if err != nil {
		log.Fatalf("api: --api-broker %q is not host:port: %v", broker, err)
	}
	bp, err := strconv.Atoi(portStr)
	if err != nil {
		log.Fatalf("api: --api-broker port %q: %v", portStr, err)
	}
	cloud.ServeCloudAPI(cloud.NewCloudAPI(dir, host, bp, ca), addr, stop)
}

func writeJSON(w http.ResponseWriter, code int, v any) {
	w.Header().Set("Content-Type", "application/json")
	w.WriteHeader(code)
	_ = json.NewEncoder(w).Encode(v)
}

func (s *server) handleIndex(w http.ResponseWriter, r *http.Request) {
	if r.URL.Path != "/" && !strings.HasPrefix(r.URL.Path, "/index") {
		writeJSON(w, http.StatusNotFound, map[string]string{"error": "not found"})
		return
	}
	btns, _ := json.Marshal(Buttons)
	page := strings.Replace(pageHTML, "__BTNS__", string(btns), 1)
	w.Header().Set("Content-Type", "text/html; charset=utf-8")
	_, _ = w.Write([]byte(page))
}

func (s *server) handleStatus(w http.ResponseWriter, r *http.Request) {
	host, _, sn, uid := s.client.Identity()
	cs := s.cloud.State()
	out := map[string]any{
		"host":       host,
		"device_sn":  sn,
		"has_user":   uid != "",
		"discovered": s.disc.Last(),
		"uptime":     time.Since(s.start).Truncate(time.Second).String(),
		// x10-ctl's own broker client, normally off. Kept for the case where
		// somebody turns it back on for wash/descale/cut-hair.
		"mqtt":        cs.Bus,
		"battery":     cs.Battery,
		"work_status": cs.Status,
		"local":       s.local.Status(),
	}
	// The bus wins when we have it. DP 163 arrives only on a percentage change
	// and only to whoever was attached at the time, so it is routinely absent;
	// the chassis reports every 2 s no matter what.
	// Same story for work status. DP 153 reports on change and only to whoever
	// is attached, so it had to be polled; StatusUpdateNotify holds the last
	// report on the bus regardless of who was listening.
	if st, derived, ok := s.local.WorkStatusDetail(); ok {
		out["work_status"] = st
		out["work_status_source"] = "bus"
		if derived {
			// The report said something the charge state contradicts. Say which
			// one is being shown rather than quietly picking one.
			out["work_status_source"] = "charge"
			out["work_status_reported"], _ = s.local.ReportedWorkStatus()
		}
	} else if cs.Status != "" {
		out["work_status_source"] = "mqtt"
	}
	// The robot's own cloud link, which is a different thing from ours and the
	// one actually worth looking at: it is how the robot gets its clock.
	if v, ok := s.local.CloudLink(); ok {
		out["robot_cloud"] = v
	}
	if b := s.local.Battery(); b.Valid {
		out["battery"] = b.Level
		out["battery_detail"] = b
		out["battery_source"] = "bus"
	} else if cs.Battery >= 0 {
		out["battery_source"] = "mqtt"
	}
	if !s.client.Configured() {
		out["ok"] = false
		out["error"] = fmt.Sprintf("waiting for a discovery broadcast on udp/%d", client.DiscoveryPort)
		writeJSON(w, http.StatusOK, out)
		return
	}
	pi, err := s.client.ProductInfo()
	if err != nil {
		out["ok"] = false
		out["error"] = err.Error()
		writeJSON(w, http.StatusOK, out)
		return
	}
	out["ok"] = true
	// Field8 is deliberately not exposed - see ProductInfo in client.go.
	out["robot"] = map[string]string{
		"brand": pi.Brand, "model": pi.Model, "code": pi.Code, "name": pi.Name,
	}
	writeJSON(w, http.StatusOK, out)
}

func (s *server) handleCommand(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPost {
		writeJSON(w, http.StatusMethodNotAllowed, map[string]string{"error": "POST only"})
		return
	}
	var body struct {
		Cmd string `json:"cmd"`
		Seq uint64 `json:"seq"`
	}
	if err := json.NewDecoder(http.MaxBytesReader(w, r.Body, 4096)).Decode(&body); err != nil {
		writeJSON(w, http.StatusBadRequest, map[string]any{"ok": false, "error": "bad json"})
		return
	}

	// Data-point commands leave over MQTT or the on-device path; movement
	// leaves over 9668. The name alone decides, so the page does not have to
	// know which channel it is on.
	if fn, ok := dpCommands[strings.ToLower(body.Cmd)]; ok {
		ctx, cancel := context.WithTimeout(r.Context(), 15*time.Second)
		defer cancel()
		via, err := fn(s, ctx)
		if err != nil {
			writeJSON(w, http.StatusOK, map[string]any{"ok": false, "error": err.Error(), "bus": via})
			return
		}
		// Fire and forget either way: the robot answers with a protocol 1 report
		// on its own schedule, not with an ack on this topic.
		writeJSON(w, http.StatusOK, map[string]any{"ok": true, "command": body.Cmd, "bus": via})
		return
	}

	// Movement, in order of preference: the on-device mds channel first because
	// it needs no broker and no cloud, then MQTT, then 9668 last.
	//
	// 9668 is last because it is broken, not because it is least preferred in
	// principle. Confirmed 2026-07-30: cloud_cli logs
	// `SSL RECV: 080801120408061001` for a gohome frame and then does nothing -
	// no dispatch, no response, robot stationary. The same method as DP 152
	// runs the full chain and drove the robot to its dock.
	if method, ok := client.Methods[strings.ToLower(body.Cmd)]; ok {
		ctx, cancel := context.WithTimeout(r.Context(), 15*time.Second)
		defer cancel()

		if via, err := s.runMethod(ctx, body.Cmd, method); err == nil {
			writeJSON(w, http.StatusOK, map[string]any{
				"ok": true, "command": body.Cmd, "bus": via, "method": method,
			})
			return
		}
	}

	res, err := s.client.Command(body.Cmd, body.Seq)
	if err != nil {
		writeJSON(w, http.StatusOK, map[string]any{"ok": false, "error": err.Error()})
		return
	}
	writeJSON(w, http.StatusOK, res)
}

// handleMQTT reports the bus, the learned topics and every data point the robot
// has reported this run.
func (s *server) handleMQTT(w http.ResponseWriter, r *http.Request) {
	writeJSON(w, http.StatusOK, s.cloud.State())
}

// handleDP sends one arbitrary data point. Raw DPs take hex (a protobuf body, wrapped and
// base64ed here); Bool, Value, Enum and String DPs take a bare JSON value.
func (s *server) handleDP(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPost {
		writeJSON(w, http.StatusMethodNotAllowed, map[string]string{"error": "POST only"})
		return
	}
	var body struct {
		DP    int             `json:"dp"`
		Hex   string          `json:"hex"`
		Value json.RawMessage `json:"value"`
	}
	if err := json.NewDecoder(http.MaxBytesReader(w, r.Body, 65536)).Decode(&body); err != nil {
		writeJSON(w, http.StatusBadRequest, map[string]any{"ok": false, "error": "bad json"})
		return
	}
	if body.DP <= 0 {
		writeJSON(w, http.StatusBadRequest, map[string]any{"ok": false, "error": "dp is required"})
		return
	}

	var value any
	switch {
	case len(body.Value) > 0:
		if err := json.Unmarshal(body.Value, &value); err != nil {
			writeJSON(w, http.StatusBadRequest, map[string]any{"ok": false, "error": "bad value"})
			return
		}
	default:
		raw, err := hex.DecodeString(body.Hex)
		if err != nil {
			writeJSON(w, http.StatusBadRequest, map[string]any{"ok": false, "error": "bad hex"})
			return
		}
		value = dp.DPValue(raw)
	}

	if err := s.cloud.SendDP(map[string]any{strconv.Itoa(body.DP): value}); err != nil {
		writeJSON(w, http.StatusOK, map[string]any{"ok": false, "error": err.Error()})
		return
	}
	writeJSON(w, http.StatusOK, map[string]any{
		"ok": true, "dp": body.DP, "name": dp.DPName(body.DP), "value": value,
	})
}

const pageHTML = `<!doctype html>
<html lang="en"><head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>eufy X10 - local</title>
<style>
  :root{--bg:#f4f5f7;--card:#fff;--fg:#14161a;--muted:#6b7280;--line:#e5e7eb;
        --accent:#2563eb;--move:#b45309;--ok:#15803d;--bad:#b91c1c}
  @media(prefers-color-scheme:dark){:root{--bg:#0d0f13;--card:#171a21;--fg:#e8eaed;
        --muted:#9aa1ac;--line:#262b34;--accent:#5b9dff;--move:#f59e0b;--ok:#4ade80;--bad:#f87171}}
  *{box-sizing:border-box}
  body{margin:0;background:var(--bg);color:var(--fg);
       font:16px/1.5 system-ui,-apple-system,Segoe UI,Roboto,sans-serif;
       display:flex;justify-content:center;padding:24px 16px}
  main{width:100%;max-width:460px}
  h1{font-size:1.15rem;margin:0 0 2px}
  .sub{color:var(--muted);font-size:.85rem;margin-bottom:18px}
  .card{background:var(--card);border:1px solid var(--line);border-radius:14px;
        padding:16px;margin-bottom:16px}
  .row{display:flex;justify-content:space-between;gap:12px;padding:4px 0;font-size:.92rem}
  .row .k{color:var(--muted)} .row .v{font-weight:600;text-align:right;word-break:break-all}
  .grid{display:grid;grid-template-columns:1fr 1fr;gap:10px}
  button{font:inherit;font-weight:600;padding:16px;border-radius:12px;border:1px solid var(--line);
         background:var(--card);color:var(--fg);cursor:pointer;transition:.12s;
         user-select:none;-webkit-user-select:none;-webkit-touch-callout:none;touch-action:manipulation}
  button:hover{border-color:var(--accent)} button:active{transform:scale(.98)}
  button:disabled{opacity:.5;cursor:wait}
  button.move{border-color:var(--move);color:var(--move)}
  .full{grid-column:1/-1}
  .tag{font-size:.7rem;color:var(--move);font-weight:700}
  .tag.mq{color:var(--accent)}
  .tag.untested{color:var(--muted)}
  img{width:100%;border-radius:10px;display:block;margin-top:10px;background:var(--bg)}
  /* the map is 266x336; smoothing it turns a grid into mush */
  img.pix{image-rendering:pixelated}
  .lbl{font-weight:500;color:var(--muted);cursor:pointer;user-select:none}
  .tabs{display:flex;gap:6px;flex-wrap:wrap;margin-top:8px}
  .tabs button{padding:6px 10px;font-size:.8rem;border-radius:8px}
  .tabs button.on{border-color:var(--accent);color:var(--accent)}
  nav{display:flex;gap:6px;margin-bottom:14px;flex-wrap:wrap}
  nav button{flex:1;padding:9px 6px;font-size:.85rem;border-radius:10px}
  nav button.on{border-color:var(--accent);color:var(--accent);font-weight:700}
  .note{font-size:.78rem;color:var(--muted);margin:6px 0 2px;line-height:1.45}
  .sched{display:flex;align-items:center;gap:10px;padding:9px 0;border-top:1px solid var(--line)}
  .sched .st{font-weight:700;font-size:1.05rem;min-width:58px}
  .sched .sd{color:var(--muted);font-size:.85rem;flex:1}
  .sched .sx{font-size:.72rem;font-weight:700;color:var(--ok)}
  .sched.off{opacity:.45} .sched.off .sx{color:var(--muted)}
  .room{display:flex;align-items:center;gap:9px;padding:7px 0;border-top:1px solid var(--line);font-size:.92rem}
  .room input{width:18px;height:18px;accent-color:var(--accent)}
  .room .rid{color:var(--muted);font-size:.75rem;min-width:18px}
  .room .ord{min-width:16px;font-size:.78rem;font-weight:600;color:var(--accent);text-align:right}
  .swatch{width:13px;height:13px;border-radius:3px;flex:none}
  .dpad{display:grid;gap:8px;margin-top:10px;justify-content:center;
    grid-template-columns:repeat(3,64px);grid-template-rows:repeat(3,52px);
    grid-template-areas:". u ." "l c r" ". d ."}
  .dpad button{font-size:20px;line-height:1}
  #mlidar{width:100%;aspect-ratio:2/1.15;background:var(--bg);border-radius:10px;margin-bottom:10px}
  #mcam{width:100%;border-radius:10px;margin:8px 0}
  /* An img is draggable by default, and that gesture is the same gesture used
     to size a zone. The browser wins, so the first drag ghosted the map. */
  #map{-webkit-user-drag:none;user-select:none;-webkit-user-select:none}
  .hit{color:#ff5c5c;font-weight:600}
  #lidar{width:100%;aspect-ratio:1;background:var(--bg);border-radius:10px;margin-top:10px}
  #toast{position:fixed;left:50%;bottom:22px;transform:translateX(-50%);
         background:var(--card);border:1px solid var(--line);border-radius:10px;
         padding:10px 16px;font-size:.9rem;opacity:0;transition:.2s;pointer-events:none;
         box-shadow:0 6px 24px rgba(0,0,0,.25);max-width:90vw}
  #toast.show{opacity:1} #toast.ok{border-color:var(--ok)} #toast.bad{border-color:var(--bad)}
  .dot{display:inline-block;width:9px;height:9px;border-radius:50%;background:var(--bad);
       margin-right:7px;vertical-align:middle}
  .dot.on{background:var(--ok)}
</style></head><body><main>
  <h1>eufy X10 Pro Omni</h1>
  <div class="sub">local control - no cloud</div>
  <nav id="nav"></nav>

  <section class="page" id="p-home">
    <div class="card" id="statusCard">
      <div class="row"><span class="k"><span class="dot" id="dot"></span>Robot</span>
                       <span class="v" id="s-name">checking...</span></div>
      <div class="row"><span class="k">Model</span><span class="v" id="s-model">-</span></div>
      <div class="row"><span class="k">Serial</span><span class="v" id="s-sn">-</span></div>
      <div class="row"><span class="k">Address</span><span class="v" id="s-host">-</span></div>
      <div class="row"><span class="k">Battery</span><span class="v" id="s-bat">-</span></div>
      <div class="row"><span class="k">State</span><span class="v" id="s-state">-</span></div>
      <div class="row"><span class="k"><span class="dot" id="ldot"></span>Bus</span>
                       <span class="v" id="s-local">-</span></div>
      <div class="row"><span class="k"><span class="dot" id="mdot"></span>Robot cloud</span>
                       <span class="v" id="s-mqtt">-</span></div>
    </div>
    <div class="card"><div class="grid" id="btns"></div></div>

    <div class="card">
      <div class="row"><span class="k">Manual drive</span><span class="v" id="driveState">-</span></div>
      <div class="note">Hold a direction. Releasing brakes but stays in remote mode, so
        the next press is instant. Remote mode is remote <em>clean</em> - it runs the mop
        and fan - so it parks and lifts the mops after 4 s idle, or press &#9632; now.</div>
      <div class="dpad">
        <button class="move" data-dir="forward" style="grid-area:u">&#9650;</button>
        <button class="move" data-dir="left"    style="grid-area:l">&#9664;</button>
        <button class="move" data-dir="brake"   style="grid-area:c">&#9632;</button>
        <button class="move" data-dir="right"   style="grid-area:r">&#9654;</button>
        <button class="move" data-dir="back"    style="grid-area:d">&#9660;</button>
      </div>
    </div>
  </section>

  <section class="page" id="p-cam" hidden>
    <div class="card">
      <div class="row"><span class="k">Camera</span>
        <span class="v"><label class="lbl"><input type="checkbox" id="camLive" checked> live</label></span></div>
      <div class="row"><span class="k">Frame</span><span class="v" id="camAge">-</span></div>
      <div class="note">Opening this page asks camera_svc to stream (30 fps) and it stops
        again about 20 s after you leave. The age is shown because a stopped pipeline
        leaves its last frame in shared memory, where it looks live.</div>
      <img id="cam" alt="camera">
      <div class="row"><span class="k">Light</span>
        <span class="v"><label class="lbl"><input type="checkbox" id="camLight"> on</label></span></div>
    </div>

  </section>

  <section class="page" id="p-manual" hidden>
    <div class="card">
      <div class="row"><span class="k">Manual mode</span><span class="v" id="manState">off</span></div>
      <div class="note">Entering forces sweep-only so the mops stay up. The brush still
        turns - Fan.Suction has no off - and the previous clean type is put back when
        you leave.</div>
      <img id="mcam" alt="camera">
      <div class="row"><span class="k">Light</span>
        <span class="v"><label class="lbl"><input type="checkbox" id="mLight"> on</label></span></div>
      <div class="row"><span class="k">Contact</span><span class="v" id="mBump">-</span></div>
      <div class="row"><span class="k">Ahead</span><span class="v" id="mAhead">-</span></div>
    </div>
    <div class="card">
      <svg id="mlidar" viewBox="-1 -1 2 1.15"></svg>
      <div class="dpad">
        <button class="move" data-dir="forward" style="grid-area:u">&#9650;</button>
        <button class="move" data-dir="left"    style="grid-area:l">&#9664;</button>
        <button class="move" data-dir="brake"   style="grid-area:c">&#9632;</button>
        <button class="move" data-dir="right"   style="grid-area:r">&#9654;</button>
        <button class="move" data-dir="back"    style="grid-area:d">&#9660;</button>
      </div>
    </div>

    <div class="card">
      <div class="row"><span class="k">Spot clean</span><span class="v" id="spotState">-</span></div>
      <div class="note">Cleans where the robot stands. The command carries no coordinate, so
        the flow is drive it there and press this. With the clean type on sweep and mop it
        may visit the dock to wet the mops first.</div>
      <div class="grid">
        <button class="move full" id="spotGo">Clean this spot <span class="tag">moves</span></button>
      </div>
    </div>
  </section>

  <section class="page" id="p-map" hidden>
    <div class="card">
      <div class="row"><span class="k">Map</span><span class="v" id="mapMeta">-</span></div>
      <div class="tabs" id="mapTabs"></div>
      <div class="row"><span class="k">Overlay</span>
        <span class="v"><label class="lbl"><input type="checkbox" id="mapOverlay" checked> dock</label></span></div>
      <div class="row"><span class="k">Dock</span><span class="v" id="mapDock">-</span></div>
      <div class="row"><span class="k">Keep-out</span><span class="v" id="mapZones">-</span></div>
      <div class="row"><span class="k">Draw</span><span class="v">
        <select id="zoneKind">
          <option value="">off</option>
          <option value="forbid">no-go</option>
          <option value="nomop">no-mop</option>
          <option value="wall">virtual wall</option>
          <option value="split">split room</option>
        </select></span></div>
      <div class="row" id="splitRow" hidden><span class="k">Split which</span>
        <span class="v"><select id="splitRoom"></select></span></div>
      <div class="grid" style="margin-top:8px">
        <button id="zoneUndo">Remove last</button>
        <button class="move" id="zoneSave">Save zones</button>
      </div>
      <div class="tabs" id="mapFlip"></div>
      <div style="position:relative;display:inline-block;max-width:100%">
        <img id="map" class="pix" alt="map" draggable="false">
        <svg id="mapOv" style="position:absolute;inset:0;width:100%;height:100%;pointer-events:none"
             preserveAspectRatio="none"></svg>
      </div>
    </div>
    <div class="card">
      <div class="row"><span class="k">Rooms</span><span class="v" id="roomInfo">-</span></div>
      <div id="roomList"></div>
      <div class="note">Ticking a room appends it to the run. The number on the left is the
        order it is sent in, so tick them in the order you want them cleaned - untick and tick
        again to move a room to the end. The robot is told an explicit order per room, the same
        field the app sends.</div>
      <div class="row"><span class="k">Order</span><span class="v" id="roomOrder">none picked</span></div>
      <div class="grid" style="margin-top:10px">
        <button class="move full" id="roomGo">Clean selected <span class="tag">moves</span></button>
      </div>
    </div>
  </section>

  <section class="page" id="p-lidar" hidden>
    <div class="card">
      <div class="row"><span class="k">Lidar</span>
        <span class="v"><label class="lbl"><input type="checkbox" id="lidLive" checked> live</label></span></div>
      <div class="note">Opening this page powers the laser and it spins down again
        about 20 s after you leave. No cleaning mode, no brushes, no mop.</div>
      <div class="row"><span class="k">Scan</span><span class="v" id="lidInfo">-</span></div>
      <svg id="lidar" viewBox="-1 -1 2 2"></svg>
      <div class="row"><span class="k">Front</span><span class="v">orange spoke</span></div>
      <div class="grid" style="margin-top:10px">
        <button id="lidOff">Spin down now</button>
      </div>
    </div>
  </section>

  <section class="page" id="p-sched" hidden>
    <div class="card">
      <div class="row"><span class="k">Schedules</span><span class="v" id="schCount">-</span></div>
      <div class="note">Times come from the robot's own config; changes go out on
        app.TimerRequest, the same request API the vendor app uses.</div>
      <div id="schList"></div>
    </div>
  </section>

  <section class="page" id="p-set" hidden>
    <div class="card">
      <div class="row"><span class="k">Cleaning settings</span><span class="v" id="setState">-</span></div>
      <div class="note">Written to app.CleanSetting on the bus. The robot plays a short
        voice prompt on every change - that is its own acknowledgement, and it follows
        the voice volume.</div>
      <div id="setList"></div>
    </div>

    <div class="card">
      <div class="row"><span class="k">Clock</span><span class="v" id="clkState">-</span></div>
      <div class="note">The robot has no NTP and normally takes its time from the cloud.
        Setting it here uses this device's clock and writes the RTC, so a reboot keeps it.</div>
      <div class="row"><span class="k">RTC drift</span><span class="v" id="clkDrift">-</span></div>
      <div class="grid" style="margin-top:10px">
        <button class="move full" id="clkSync">Set from this device</button>
      </div>
    </div>

    <div class="card">
      <div class="row"><span class="k">Per-room settings</span><span class="v" id="roomSetState">-</span></div>
      <div class="note">Names commit on Enter or when you click away. Parameters are
        stored per room but ignored unless <em>custom</em> is on - with it off
        every room uses the global settings above. Order 0 means "no preference"; the robot
        picks. Written through MapEditRequest.RoomsCustom.</div>
      <div class="row"><span class="k">Custom per room</span>
        <span class="v"><label class="lbl"><input type="checkbox" id="roomCustom"> on</label></span></div>
      <div id="roomSetList"></div>
      <div class="grid" style="margin-top:10px">
        <button class="move full" id="roomSetSave">Save room settings</button>
      </div>
    </div>

    <div class="card">
      <div class="row"><span class="k">Merge rooms</span><span class="v" id="mergeState">-</span></div>
      <div class="note">Tick two or more <em>adjacent</em> rooms. The robot refuses
        non-adjacent ones. This rewrites the room map and there is no delete - merging
        is how the count comes down.</div>
      <div id="mergeList"></div>
      <div class="grid" style="margin-top:10px">
        <button class="move full" id="mergeGo">Merge selected <span class="tag">rewrites map</span></button>
      </div>
      <div class="grid" style="margin-top:8px">
        <button id="mapUndo">Undo last map edit</button>
        <button id="mapDone">Finish editing</button>
      </div>
    </div>
  </section>

  <section class="page" id="p-opts" hidden>
    <div class="card">
      <div class="row"><span class="k">Quiet hours</span><span class="v" id="dndState">-</span></div>
      <div class="note">The window the robot will not start a scheduled clean in, and will
        not go back to the dock to wash in, because both are loud. Written to
        app.UndisturbedSetting on the bus - the same setting the app sends as data point 157.
        The window may wrap midnight.</div>
      <div class="row"><span class="k">On</span>
        <span class="v"><label class="lbl"><input type="checkbox" id="dndOn"> enabled</label></span></div>
      <div class="row"><span class="k">From</span><span class="v"><input type="time" id="dndFrom"></span></div>
      <div class="row"><span class="k">Until</span><span class="v"><input type="time" id="dndTo"></span></div>
      <div class="grid" style="margin-top:10px">
        <button class="move full" id="dndSave">Save quiet hours</button>
      </div>
    </div>

    <div class="card">
      <div class="row"><span class="k">Auto empty</span><span class="v" id="dustState">-</span></div>
      <div class="note">Whether the station empties the bin into the bag on its own after a
        clean. The <em>Empty dustbin</em> button on the Control page is the one-shot version
        and is a different thing. A running mop dry blocks both.</div>
      <div class="row"><span class="k">On</span>
        <span class="v"><label class="lbl"><input type="checkbox" id="dustOn"> enabled</label></span></div>
      <div class="row"><span class="k">How often</span><span class="v" id="dustHow">-</span></div>
      <div class="row"><span class="k">Trigger</span>
        <span class="v"><select id="dustMode">
          <option value="0">by task</option><option value="1">by time</option></select></span></div>
      <div class="row"><span class="k">Every N cleans</span>
        <span class="v"><input type="number" id="dustTask" min="1" max="100" style="width:80px"></span></div>
      <div class="row"><span class="k">Or every N minutes</span>
        <span class="v"><input type="number" id="dustMin" min="0" max="1440" step="10" style="width:80px"></span></div>
      <div class="grid" style="margin-top:10px">
        <button class="move full" id="dustSave">Save empty frequency</button>
      </div>
    </div>

    <div class="card">
      <div class="row"><span class="k">Mop wash</span><span class="v" id="washState">-</span></div>
      <div class="note">How often the station backwashes the mop during a run. The interval is
        minutes when the trigger is by time and square metres when it is by area. Off is not
        settable from here - use the app.</div>
      <div class="row"><span class="k">During a run</span><span class="v" id="washMission">-</span></div>
      <div class="row"><span class="k">Trigger</span>
        <span class="v"><select id="washMode"><option value="0">by partition</option>
          <option value="1">by time</option><option value="2">by area</option></select></span></div>
      <div class="row"><span class="k">Interval</span>
        <span class="v"><input type="number" id="washInterval" min="1" max="1000" style="width:80px">
          <span id="washUnit"></span></span></div>
      <div class="row"><span class="k">Duration</span>
        <span class="v"><select id="washDur"><option value="0">short</option>
          <option value="1">medium</option><option value="2">long</option></select></span></div>
      <div class="grid" style="margin-top:10px">
        <button class="move full" id="washSave">Save wash frequency</button>
      </div>
    </div>

    <div class="card">
      <div class="row"><span class="k">Mop dry</span><span class="v" id="dryState">-</span></div>
      <div class="note">Runs after a mop clean and owns the dock for up to two hours, which is
        what blocks an auto empty. Off cannot be encoded on the bus - turn it off in the app.
        <em>Dry mop</em> on the Control page starts one now.</div>
      <div class="row"><span class="k">Mode</span>
        <span class="v"><select id="dryCfg"><option value="1">standard</option>
          <option value="2">quick</option></select></span></div>
      <div class="row"><span class="k">Duration</span>
        <span class="v"><select id="dryDur"><option value="0">short</option>
          <option value="1">medium</option><option value="2">long</option></select></span></div>
      <div class="row"><span class="k">Dock</span><span class="v" id="dryDock">-</span></div>
      <div class="grid" style="margin-top:10px">
        <button class="move full" id="drySave">Save dry setting</button>
        <button id="dryStop">Stop the running dry</button>
      </div>
    </div>

    <div class="card">
      <div class="row"><span class="k">Off-peak charging</span><span class="v" id="peakState">-</span></div>
      <div class="note">The window the robot is allowed to charge in, written to
        app.Unisetting. The window may wrap midnight.</div>
      <div class="row"><span class="k">On</span>
        <span class="v"><label class="lbl"><input type="checkbox" id="peakOn"> enabled</label></span></div>
      <div class="row"><span class="k">From</span><span class="v"><input type="time" id="peakFrom"></span></div>
      <div class="row"><span class="k">Until</span><span class="v"><input type="time" id="peakTo"></span></div>
      <div class="grid" style="margin-top:10px">
        <button class="move full" id="peakSave">Save charging window</button>
      </div>
    </div>

    <div class="card">
      <div class="row"><span class="k">Voice volume</span><span class="v" id="volState">-</span></div>
      <div class="note">Written to app.VoiceSetting on the bus. Most other writes make the
        robot say something; this is the one that decides how loudly. 0 is silent.</div>
      <div class="row"><span class="k">Level</span>
        <span class="v"><input type="range" id="volRange" min="0" max="100" step="5"
          style="width:150px"> <span id="volNum">-</span></span></div>
      <div class="grid" style="margin-top:10px">
        <button class="move full" id="volSave">Set volume</button>
      </div>
    </div>

    <div class="card">
      <div class="row"><span class="k">Switches</span><span class="v" id="uniState">-</span></div>
      <div class="note">Written to app.Unisetting, one field per switch, and only the ones
        you change are sent. A switch shown as <em>unknown</em> has no key in
        user_config.yaml, so its current value cannot be read back - it can still be set.</div>
      <div id="uniList"></div>
      <div class="row"><span class="k">Dust bag reminder</span>
        <span class="v"><input type="number" id="uniDust" min="0" step="1000" style="width:110px"></span></div>
      <div class="grid" style="margin-top:10px">
        <button class="move full" id="uniDustSave">Save reminder threshold</button>
      </div>
    </div>

    <div class="card">
      <div class="row"><span class="k">Consumables</span><span class="v" id="conState">-</span></div>
      <div class="note">Read-only. Two counters per part: the full life of the part and what
        is left of it, both in minutes. Nothing here resets anything - a reset that fires by
        accident cannot be undone.</div>
      <div id="conList"></div>
      <div id="conTotals" class="note" style="margin-top:10px"></div>
    </div>
  </section>

  <section class="page" id="p-bus" hidden>
    <div class="card">
      <div class="row"><span class="k">mds channel</span>
        <span class="v"><select id="busChan"></select></span></div>
      <div class="note">The newest record on a channel, decoded as far as the wire allows.
        There are no field names on this transport, only numbers, so a field is identified by
        watching which number moves when something changes. Read-only: every read is an
        SHM_RDONLY attach and nothing here publishes.
        <b>Records are partial</b> - a channel emits only what changed, and proto3 omits
        zeros, so an absent field means "not in this record", never "zero".</div>
      <div class="row"><span class="k">State</span><span class="v" id="busState">-</span></div>
      <div class="grid" style="margin-top:8px">
        <button id="busRead">Read newest</button>
        <label class="lbl"><input type="checkbox" id="busLive"> follow, 1 s</label>
      </div>
      <pre id="busOut" style="overflow-x:auto;margin-top:10px"></pre>
    </div>
  </section>

  <div id="toast"></div>
</main><script>
const PAGES=[['home','Control'],['cam','Camera'],['manual','Manual'],['map','Map'],['lidar','Lidar'],['sched','Schedules'],['set','Settings'],['opts','Options'],['bus','Bus']];
const nav=document.getElementById('nav');
let current='home';
function show(id){
  // Leaving a page while a direction is held has to stop the robot. The blur
  // and visibilitychange handlers only catch losing the whole tab, and both
  // D-pads live on pages you can navigate away from mid-press.
  if(typeof driveRelease==='function') driveRelease();
  current=id;
  PAGES.forEach(([k])=>document.getElementById('p-'+k).hidden = k!==id);
  [...nav.children].forEach(b=>b.className = b.dataset.k===id?'on':'');
  if(typeof manualPage==='function') manualPage(id==='manual');
  if(id==='cam') camTick();
  if(id==='map') loadMap();
  if(id==='sched') loadSchedules();
  if(id==='lidar') loadLidar();
  if(id==='set'){ loadSettings(); loadRoomSettings(); loadClock(); }
  if(id==='opts'){ loadDnd(); loadDust(); loadWash(); loadDry(); loadPeak();
                   loadVolume(); loadUni(); loadConsumables(); }
  busPage(id==='bus');
}
PAGES.forEach(([k,label])=>{
  const b=document.createElement('button');
  b.textContent=label; b.dataset.k=k; b.className=k==='home'?'on':'';
  b.onclick=()=>show(k); nav.appendChild(b);
});

const BTNS = __BTNS__;
const btnWrap = document.getElementById('btns');
BTNS.forEach((b,i)=>{
  const el=document.createElement('button');
  el.className=(b.move?'move ':'')+(i===0?'full':'');
  // the bus tag is not decoration: an mqtt button fails while only the local
  // path is up, and the error says so, but the tag says so before the tap.
  el.innerHTML=b.label+(b.move?' <span class="tag">moves</span>':'')
    +(b.bus!=='bus'?' <span class="tag mq">'+b.bus+'</span>':'')
    +(b.untested?' <span class="tag untested">untested</span>':'');
  el.onclick=()=>send(b.cmd,el);
  btnWrap.appendChild(el);
});
function toast(msg,cls){const t=document.getElementById('toast');t.textContent=msg;
  t.className='show '+(cls||'');setTimeout(()=>t.className=t.className.replace('show',''),2600);}
async function send(cmd,el){
  el.disabled=true;
  try{const r=await fetch('/api/command',{method:'POST',headers:{'Content-Type':'application/json'},
        body:JSON.stringify({cmd})});
      const j=await r.json();
      // 9668 stopped acking after the 2026-07-26 re-pair but still obeyed, so a
      // missing ack is "sent", not a failure. Only an explicit reject is red.
      if(j.ok&&j.ack==='OK') toast(cmd+': OK','ok');
      else if(j.ok) toast(cmd+' via '+(j.bus||'?'),'ok');
      else toast(cmd+': '+(j.text||j.error||j.reply||'rejected'),'bad');
  }catch(e){toast(cmd+': '+e,'bad');}
  el.disabled=false;
}
function set(id,v){const e=document.getElementById(id); if(e) e.textContent=v||'-';}
async function refresh(){
  try{const j=await (await fetch('/api/status')).json();
     set('s-sn', j.device_sn); set('s-host', j.host);
     const bd=j.battery_detail;
     let bat = j.battery>=0 ? j.battery+'%' : '-';
     if(bd&&bd.valid){
       bat += ' \u00b7 '+(bd.millivolts/1000).toFixed(1)+' V';
       // charge_state 1 is a charge in progress, 2 is on the dock with the pack
       // full. Both mean docked; only one of them means current is flowing.
       if(bd.charge_state===1) bat += ' \u00b7 charging';
       else if(bd.charge_state>0) bat += ' \u00b7 on dock';
       if(bd.charge_error||bd.discharge_error||bd.charging_chip_error) bat += ' \u00b7 FAULT';
     } else if(j.battery_source==='mqtt') bat += ' (dp)';
     set('s-bat', bat);
     // A state derived from the charge state disagrees with the last report,
     // and which one is on screen is worth saying out loud.
     set('s-state', j.work_status
       + (j.work_status_source==='charge'
          ? ' \u00b7 from charge state, last report said ' + (j.work_status_reported||'nothing')
          : ''));
     const lo=j.local||{};
     document.getElementById('ldot').classList.toggle('on', !!(lo.battery&&lo.battery.valid));
     set('s-local', !lo.available ? 'off-device'
       : (lo.battery&&lo.battery.valid ? 'chassis bus live' : (lo.last_error||'waiting')));
     // The robot's own cloud link, not ours. Ours is normally off and saying so
     // told nobody anything; this is the one that matters, because it is how the
     // robot gets its clock.
     const rc=j.robot_cloud, m=j.mqtt||{};
     document.getElementById('mdot').classList.toggle('on', rc===2);
     let cl = rc===undefined ? 'unknown' : (rc===2?'connected':'state '+rc);
     if(m.mode&&m.mode!=='off') cl+=' \u00b7 x10-ctl '+(m.connected?'joined':'joining');
     set('s-mqtt', cl);
     if(j.ok){document.getElementById('dot').classList.add('on');
       set('s-name', (j.robot&&j.robot.name)||'online');
       set('s-model', (j.robot&&j.robot.model));}
     else{document.getElementById('dot').classList.remove('on');
       set('s-name', j.error||'unreachable');}
  }catch(e){set('s-name','unreachable');}
}

// --- camera. Polled at 2 fps only while its page is open, so a phone left on
// the map page is not quietly encoding JPEGs on the robot's CPU.
const camImg=document.getElementById('cam'), camLive=document.getElementById('camLive');
let camTimer=null;
function camShot(){
  camImg.src='/api/camera.jpg?scale=3&q=70&t='+Date.now();
  fetch('/api/camera').then(r=>r.json()).then(m=>{
    if(!m.available){ set('camAge', m.error||'no segment'); return; }
    const a=m.age_seconds;
    set('camAge', a==null ? m.width+'x'+m.height
      : (a<5 ? 'live \u00b7 '+m.width+'x'+m.height
             : 'STALE \u00b7 '+(a<120? a+' s' : Math.floor(a/60)+' min')+' old'));
  }).catch(()=>{});
}
// The repeat used to be started only by the checkbox's onchange handler, which
// never fires when the box is checked in the markup - so the page took one
// frame and then sat there until you toggled it off and on. Same shape of bug
// as the lidar's, and the same fix: one function, called on load and on every
// page switch as well as on change.
function camTick(){
  clearInterval(camTimer); camTimer=null;
  if(!camLive.checked) return;
  camShot();
  camTimer=setInterval(()=>{ if(current==='cam') camShot(); },500);
}
camLive.onchange=camTick;
camTick();

// Supplement LED. The channel takes a 0..100 level and the robot accepts one -
// it logs CtrlLight level=60, enable=1 - but the LED itself does not dim,
// checked by eye. So this is a switch, and the level is only ever 0 or 100.
const camLight=document.getElementById('camLight');
camLight.onchange=()=>{
  fetch('/api/camera/light?level='+(camLight.checked?100:0),{method:'POST'})
    .then(r=>r.json())
    .then(j=>{ if(!j.ok) toast(j.error||'light failed'); })
    .catch(()=>toast('light error'));
};

// --- map
// Layers come from /api/map, which reports each one's own frame. Cliff is
// 172x229 and coverage 174x232 against the map's 266x336, so the dock, the
// robot and every zone are map-frame only.
let mapLoaded=false, mapLayer='house', mapLayerMain=true;
function drawMap(){
  document.getElementById('map').src='/api/map.png?layer='+mapLayer+'&flip='+mapFlipMode+'&t='+Date.now();
  drawOverlay();
}

// Overlay: the dock and the robot's trail, in map pixels on top of the PNG.
//
// This is the test for the coordinate transform and it needs no driving. The
// dock's world position is known from dock_pose.yaml, so if the marker lands on
// the dock in the image, world-to-pixel is right. If it lands somewhere else,
// the flip applied to the PNG and the transform disagree - which is the thing
// most likely to be wrong, since the map is rendered mirrored.
function drawOverlay(){
  const svg=document.getElementById('mapOv');
  if(!document.getElementById('mapOverlay').checked){ svg.innerHTML=''; set('mapDock','overlay off'); return; }
  // Cliff and coverage have their own size and origin. Drawing map-frame
  // pixels on them puts the dock in the wrong room, so draw nothing and say so.
  if(!mapLayerMain){ svg.innerHTML=''; set('mapDock','this layer has its own frame');
                     set('mapZones','map frame only'); return; }
  fetch('/api/pose').then(r=>r.json()).then(j=>{
    if(!j.ok){ svg.innerHTML=''; set('mapDock', j.error||'unavailable'); return; }
    const f=j.frame, W=f.width, H=f.height;
    svg.setAttribute('viewBox','0 0 '+W+' '+H);
    // The PNG is served flipped; the coordinates are not. Mirror x to match.
    const fx = (mapFlipMode==='h'||mapFlipMode==='both') ? (x=>W-x) : (x=>x);
    const fy = (mapFlipMode==='v'||mapFlipMode==='both') ? (y=>H-y) : (y=>y);
    // No path. The 0x92d203d6 records are not the pixel tuples they look like -
    // most fall outside the map and the few that do not draw somewhere wrong.
    // Drawing a dozen mystery points is worse than drawing none, so the trail
    // stays off until the format is actually understood.
    let g='';
    // Keep-out geometry, in world metres, through the same transform the dock
    // marker validated.
    zoneFrame=f;
    const build=zones=>{
      let zg='';
      zones.forEach(zn=>{
        const pts=zn.points.map(p=>{
          const px=(p.x-f.origin_x)/f.resolution, py=(p.y-f.origin_y)/f.resolution;
          return fx(px).toFixed(1)+','+fy(py).toFixed(1);
        }).join(' ');
        if(zn.kind==='wall')
          zg+='<polyline points="'+pts+'" fill="none" stroke="#ff5c5c" stroke-width="2"/>';
        else
          zg+='<polygon points="'+pts+'" fill="'+(zn.kind==='nomop'?'#4c9fff':'#ff5c5c')
             +'" fill-opacity="0.25" stroke="'+(zn.kind==='nomop'?'#4c9fff':'#ff5c5c')
             +'" stroke-width="1.2"/>';
      });
      set('mapZones', (zones.length ? zones.length+' zone(s)' : 'none')
        +(zoneDraft?' \u00b7 unsaved':''));
      return zg;
    };
    // The robot, but only when the pose is in the map frame.
    if(j.pose_px){
      g+='<circle cx="'+fx(j.pose_px.x).toFixed(1)+'" cy="'+fy(j.pose_px.y).toFixed(1)
        +'" r="4.5" fill="#ff8c42" stroke="#000" stroke-width="0.7"/>';
    }
    if(j.dock){
      g+='<circle cx="'+fx(j.dock.px).toFixed(1)+'" cy="'+fy(j.dock.py).toFixed(1)
        +'" r="4" fill="none" stroke="#5ad68a" stroke-width="1.6"/>';
      set('mapDock','pixel '+j.dock.px.toFixed(0)+', '+j.dock.py.toFixed(0)
        +(j.pose ? ' \u00b7 robot '+j.pose.frame+' frame' : ''));
    }
    // One assignment, at the end, with the zones already in it. Two of them was
    // the bug: zones went in first and the second assignment wiped them, so a
    // freshly drawn zone was reported saved and never appeared.
    const paint=zg=>{ svg.innerHTML=zg+g; };
    // While editing, draw the working copy - otherwise a redraw would silently
    // discard zones the user has added but not yet saved.
    if(zoneDraft&&zoneList) paint(build(zoneList));
    else {
      paint('');
      fetch('/api/zones').then(r=>r.json()).then(z=>{
        if(z.ok&&z.zones){ zoneList=z.zones; paint(build(z.zones)); }
      }).catch(()=>{});
    }
  }).catch(()=>set('mapDock','error'));
}
document.getElementById('mapOverlay').onchange=drawOverlay;
zoneMapEvents();
// room palette must match roomColours in view.go so the list reads as a legend
const ROOM_RGB=['#0000','#4c9fff','#ff8c42','#5ad68a','#e06cd8','#f2d04b','#6be0d8','#b080ff','#a0a8b4'];
let roomsLoaded=false;
// Cleaning order is the order rooms were ticked, not the order they happen to
// sit in the list. The encoder has always sent an explicit per-room order -
// SelectRoomsClean carries Room{id, order} - but this page collected the ticks
// with querySelectorAll(':checked'), which returns DOM order, so whatever the
// user clicked first was thrown away before it reached the request.
let roomPick=[];
function roomOrderPaint(){
  const names={};
  document.querySelectorAll('#roomList .room').forEach(d=>{
    const id=+d.querySelector('input').value;
    names[id]=d.dataset.name||('room '+id);
    d.querySelector('.ord').textContent = roomPick.indexOf(id)<0 ? '' : (roomPick.indexOf(id)+1);
  });
  set('roomOrder', roomPick.length ? roomPick.map((id,i)=>(i+1)+'. '+names[id]).join('  \u00b7  ')
                                   : 'none picked');
}
function roomToggle(id,on){
  roomPick=roomPick.filter(x=>x!==id);
  if(on) roomPick.push(id);
  roomOrderPaint();
}
function loadRooms(){
  if(roomsLoaded) return;
  fetch('/api/rooms').then(r=>r.ok?r.json():Promise.reject()).then(j=>{
    roomsLoaded=true;
    roomPick=[];
    set('roomInfo', (j.map_name||'map')+' \u00b7 #'+j.map_id);
    const list=document.getElementById('roomList'); list.innerHTML='';
    j.rooms.forEach(rm=>{
      const d=document.createElement('label'); d.className='room'; d.dataset.name=rm.name;
      d.innerHTML='<span class="ord"></span><input type="checkbox" value="'+rm.id+'">'
        +'<span class="swatch" style="background:'+(ROOM_RGB[rm.id]||'#888')+'"></span>'
        +'<span class="rid">'+rm.id+'</span><span>'+rm.name+'</span>';
      d.querySelector('input').onchange=ev=>roomToggle(rm.id, ev.target.checked);
      list.appendChild(d);
    });
    roomOrderPaint();
  }).catch(()=>set('roomInfo','unavailable'));
}
document.getElementById('roomGo').onclick=async(e)=>{
  const ids=roomPick.slice();
  if(!ids.length){ toast('pick at least one room','bad'); return; }
  e.target.disabled=true;
  try{
    const r=await fetch('/api/roomclean',{method:'POST',headers:{'Content-Type':'application/json'},
      body:JSON.stringify({rooms:ids})});
    const j=await r.json();
    toast(j.ok?('cleaning '+ids.length+' room(s) via '+j.bus):(j.error||'rejected'), j.ok?'ok':'bad');
  }catch(err){ toast(''+err,'bad'); }
  e.target.disabled=false;
};

function loadMap(){
  loadRooms();
  if(mapLoaded){ drawMap(); return; }
  fetch('/api/map').then(r=>r.ok?r.json():Promise.reject()).then(doc=>{
    mapLoaded=true;
    const md=doc.metaData||{}, m=md.x10Frame||{};
    set('mapMeta','#'+md.vendorMapId+' \u00b7 '+m.width+'x'+m.height+' \u00b7 '+m.resolution_m_per_px.toFixed(3)+' m/px');
    const wrap=document.getElementById('mapTabs'); wrap.innerHTML='';
    (md.x10Layers||[]).forEach((L,i)=>{
      const b=document.createElement('button');
      b.textContent=L.label; b.className=i===0?'on':'';
      b.title=(L.width&&L.height?L.width+'x'+L.height:'')+(L.main_frame?'':' · own frame');
      b.onclick=()=>{ [...wrap.children].forEach(c=>c.className=''); b.className='on';
                      mapLayer=L.key; mapLayerMain=L.main_frame!==false; drawMap(); drawOverlay(); };
      wrap.appendChild(b);
      if(i===0){ mapLayer=L.key; mapLayerMain=L.main_frame!==false; }
    });
    drawMap();
  }).catch(()=>set('mapMeta','no map on disk'));
}

// --- schedules
const DAYS=['Sun','Mon','Tue','Wed','Thu','Fri','Sat'];
function schAction(body,label){
  fetch('/api/schedule',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify(body)})
    .then(r=>r.json())
    .then(j=>{ toast(j.ok?label:(j.error||'failed')); loadSchedules(); })
    .catch(()=>toast('error'));
}
// Room names and the room list both live on the settings payload, and the room
// picker cannot be drawn without them, so this waits rather than racing.
function loadSchedules(){
  const ready = roomSetView ? Promise.resolve()
    : fetch('/api/roomsettings').then(r=>r.json())
        .then(j=>{ if(j.ok) roomSetView=j.view; }).catch(()=>{});
  ready.then(drawSchedules);
}
function drawSchedules(){
  fetch('/api/schedules').then(r=>r.ok?r.json():Promise.reject()).then(j=>{
    const list=document.getElementById('schList'); list.innerHTML='';
    set('schCount', j.schedules.length+' configured');
    j.schedules.forEach(s=>{
      const d=document.createElement('div'); d.className='sched';
      const days=(s.days&&s.days.length)?s.days.join(' '):'once';
      // Room ids mean nothing on their own; the names come from the map, which
      // the settings page already loads.
      const rn=(s.rooms||[]).map(id=>{
        const r=(roomSetView&&roomSetView.rooms||[]).find(x=>x.id===id);
        return r?r.name:('#'+id);
      });
      d.innerHTML='<div class="st">'+s.at+'</div><div class="sd">'+days+'</div>'
        +'<div class="sx"><button data-id="'+s.id+'" data-act="'
        +(s.enabled?'disable':'enable')+'">'+(s.enabled?'on':'off')+'</button>'
        +' <button data-edit="'+s.id+'">edit</button></div>';
      if(!s.enabled) d.classList.add('off');
      list.appendChild(d);

      // Editor, hidden until asked for. week_bits is a Sunday-first bitmask -
      // bit 0 Sunday - which matches the two schedules already on the robot
      // (18 = Mon+Thu, 36 = Tue+Fri).
      const ed=document.createElement('div');
      ed.className='card'; ed.style.margin='6px 0'; ed.style.padding='8px 10px'; ed.hidden=true;
      ed.innerHTML='<div class="row"><span class="k">Time</span><span class="v">'
        +'<input type="time" id="st'+s.id+'" value="'+s.at+'"></span></div>'
        +'<div class="row"><span class="k">Days</span><span class="v">'
        +DAYS.map((n,i)=>'<label class="lbl" style="margin-left:6px"><input type="checkbox" '
          +'class="dow'+s.id+'" value="'+i+'"'+((s.week_bits>>i)&1?' checked':'')+'> '+n+'</label>').join('')
        +'</span></div>'
        +'<div class="row"><span class="k">Rooms</span><span class="v" id="rmc'+s.id+'">-</span></div>'
        +'<div class="note">Tick to add. The number is the cleaning order, and the '
        +'arrows change it. The robot cleans them in the order shown.</div>'
        +'<div id="rml'+s.id+'"></div>'
        +'<div class="grid" style="margin-top:8px"><button data-rooms="'+s.id+'">Save rooms</button></div>'
        +'<div class="grid" style="margin-top:8px"><button data-save="'+s.id+'">Save time</button>'
        +'<button data-skip="'+s.id+'">Skip next</button>'
        +'<button data-del="'+s.id+'">Delete</button></div>';
      list.appendChild(ed);
      d.querySelector('[data-edit]').onclick=()=>{ ed.hidden=!ed.hidden; };

      ed.querySelector('[data-save]').onclick=()=>{
        const t=document.getElementById('st'+s.id).value.split(':');
        let bits=0;
        ed.querySelectorAll('.dow'+s.id+':checked').forEach(c=>{ bits|=1<<(+c.value); });
        schAction({id:s.id,action:'retime',hour:+t[0],minute:+t[1],week_bits:bits},'time saved');
      };
      // Order is the whole point: the robot cleans in the order the list is
      // sent, so the picked array IS the setting and the ticks only edit it.
      const picked=[...(s.rooms||[])];
      const roomName=id=>{
        const r=(roomSetView&&roomSetView.rooms||[]).find(x=>x.id===id);
        return r?r.name:('#'+id);
      };
      const drawRooms=()=>{
        const box=ed.querySelector('#rml'+s.id); if(!box) return;
        const all=(roomSetView&&roomSetView.rooms||[]);
        set('rmc'+s.id, picked.length ? picked.length+' selected'
                                      : 'none - this schedule would clean nothing');
        let h='';
        picked.forEach((id,i)=>{
          h+='<div class="room"><input type="checkbox" data-drop="'+id+'" checked>'
            +'<span class="swatch" style="background:'+(ROOM_RGB[id]||'#888')+'"></span>'
            +'<span class="rid">'+(i+1)+'</span><span style="flex:1">'+roomName(id)+'</span>'
            +'<button data-up="'+id+'"'+(i===0?' disabled':'')+'>↑</button>'
            +'<button data-down="'+id+'"'+(i===picked.length-1?' disabled':'')+'>↓</button></div>';
        });
        all.filter(r=>picked.indexOf(r.id)<0).forEach(r=>{
          h+='<div class="room" style="opacity:.65"><input type="checkbox" data-add="'+r.id+'">'
            +'<span class="swatch" style="background:'+(ROOM_RGB[r.id]||'#888')+'"></span>'
            +'<span class="rid">-</span><span style="flex:1">'+r.name+'</span></div>';
        });
        box.innerHTML=h || '<div class="note">No rooms on the map yet.</div>';
        box.querySelectorAll('[data-add]').forEach(c=>{
          c.onchange=()=>{ picked.push(+c.dataset.add); drawRooms(); };
        });
        box.querySelectorAll('[data-drop]').forEach(c=>{
          c.onchange=()=>{ const i=picked.indexOf(+c.dataset.drop);
                           if(i>=0) picked.splice(i,1); drawRooms(); };
        });
        const swap=(i,j)=>{ const t=picked[i]; picked[i]=picked[j]; picked[j]=t; drawRooms(); };
        box.querySelectorAll('[data-up]').forEach(b=>{
          b.onclick=()=>{ const i=picked.indexOf(+b.dataset.up); if(i>0) swap(i,i-1); };
        });
        box.querySelectorAll('[data-down]').forEach(b=>{
          b.onclick=()=>{ const i=picked.indexOf(+b.dataset.down);
                          if(i>=0&&i<picked.length-1) swap(i,i+1); };
        });
      };
      drawRooms();
      ed.querySelector('[data-rooms]').onclick=()=>{
        if(!picked.length){ toast('pick at least one room','bad'); return; }
        fetch('/api/schedulerooms',{method:'POST',headers:{'Content-Type':'application/json'},
          body:JSON.stringify({id:s.id,rooms:picked.map(id=>({id:id}))})})
          .then(r=>r.json())
          .then(j=>{ toast(j.ok?('rooms saved, cleaning order '+picked.join(' → ')):(j.error||'failed'));
                     loadSchedules(); })
          .catch(()=>toast('error'));
      };
      ed.querySelector('[data-skip]').onclick=()=>schAction({id:s.id,action:'skip'},'skipping next run');
      ed.querySelector('[data-del]').onclick=()=>{
        if(confirm('Delete schedule '+s.id+' ('+s.at+')?'))
          schAction({id:s.id,action:'delete'},'deleted');
      };
    });
    list.querySelectorAll('button[data-id]').forEach(b=>{
      b.onclick=()=>{
        b.disabled=true;
        fetch('/api/schedule',{method:'POST',headers:{'Content-Type':'application/json'},
          body:JSON.stringify({id:+b.dataset.id,action:b.dataset.act})})
          .then(r=>r.json()).then(j=>{
            toast(j.ok?('schedule '+b.dataset.id+' '+b.dataset.act+'d'):(j.error||'failed'));
            loadSchedules();
          }).catch(()=>{ toast('error'); b.disabled=false; });
      };
    });
  }).catch(()=>set('schCount','unavailable'));
}

// --- cleaning settings. Each control writes only its own field: the robot
// treats an absent submessage as "no change", so sending the whole set every
// time would overwrite values the user never touched.
const SET_FIELDS=[['clean_type','Mode'],['fan','Suction'],
                  ['mop_water','Water'],['clean_extent','Extent']];
function loadSettings(){
  fetch('/api/settings').then(r=>r.json()).then(j=>{
    const box=document.getElementById('setList'); box.innerHTML='';
    if(!j.ok){ set('setState', j.error||'unavailable'); return; }
    set('setState','on the bus');
    SET_FIELDS.forEach(([key,label])=>{
      const cur=j.settings[key], names=j.names[key]||[];
      const row=document.createElement('div'); row.className='row';
      const opts=names.map((n,i)=>'<option value="'+i+'"'+(i===cur?' selected':'')+'>'
        +n.replace(/_/g,' ')+'</option>').join('');
      row.innerHTML='<span class="k">'+label+'</span><span class="v">'
        +'<select data-key="'+key+'">'+opts+'</select></span>';
      box.appendChild(row);
    });
    box.querySelectorAll('select[data-key]').forEach(sel=>{
      sel.onchange=()=>{
        const body={}; body[sel.dataset.key]=+sel.value;
        sel.disabled=true;
        fetch('/api/settings',{method:'POST',headers:{'Content-Type':'application/json'},
          body:JSON.stringify(body)})
          .then(r=>r.json())
          .then(j=>{ toast(j.ok?(sel.dataset.key+' set'):(j.error||'failed')); loadSettings(); })
          .catch(()=>{ toast('error'); sel.disabled=false; });
      };
    });
  }).catch(()=>set('setState','error'));
}

// --- quiet hours. Three values that only mean anything together, so this one
// posts on a button rather than on change: half a window is not a setting.
function hhmm(t){ return String(t.hour).padStart(2,'0')+':'+String(t.minute).padStart(2,'0'); }
function loadDnd(){
  fetch('/api/undisturbed').then(r=>r.json()).then(j=>{
    if(!j.ok){ set('dndState', j.error||'unavailable'); return; }
    const u=j.undisturbed;
    set('dndState', (u.enabled?j.window:'off') + (j.active_now?' · quiet now':''));
    document.getElementById('dndOn').checked=!!u.enabled;
    document.getElementById('dndFrom').value=hhmm(u.begin);
    document.getElementById('dndTo').value=hhmm(u.end);
  }).catch(()=>set('dndState','error'));
}
document.getElementById('dndSave').onclick=()=>{
  const b=document.getElementById('dndSave');
  const parse=id=>{
    const v=document.getElementById(id).value||'00:00', p=v.split(':');
    return {hour:+p[0], minute:+p[1]};
  };
  b.disabled=true;
  fetch('/api/undisturbed',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({enabled:document.getElementById('dndOn').checked,
                         begin:parse('dndFrom'), end:parse('dndTo')})})
    .then(r=>r.json())
    .then(j=>{ toast(j.ok?'quiet hours set':(j.error||'failed')); loadDnd(); })
    .catch(()=>toast('error'))
    .finally(()=>{ b.disabled=false; });
};

// --- automatic dust emptying. Writes on change, like the other switches;
// it is one boolean and a confirmation step would just be in the way.
function loadDust(){
  fetch('/api/dust').then(r=>r.json()).then(j=>{
    if(!j.ok){ set('dustState', j.error||'unavailable'); return; }
    const d=j.dust;
    set('dustState', d.enabled?'on':'off');
    document.getElementById('dustOn').checked=!!d.enabled;
    set('dustHow', j.how||j.mode_name||('mode '+d.mode));
    document.getElementById('dustMode').value=String(d.mode);
    document.getElementById('dustTask').value=d.task;
    document.getElementById('dustMin').value=d.minutes;
  }).catch(()=>set('dustState','error'));
}
document.getElementById('dustSave').onclick=()=>{
  const b=document.getElementById('dustSave'); b.disabled=true;
  const mode=+document.getElementById('dustMode').value;
  fetch('/api/dust',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({mode, task:+document.getElementById('dustTask').value,
                         minutes:+document.getElementById('dustMin').value})})
    .then(r=>r.json())
    .then(j=>{ toast(j.ok?('empty '+(j.how||'set')):(j.error||'failed'), j.ok?'ok':'bad'); loadDust(); })
    .catch(()=>toast('error','bad'))
    .finally(()=>{ b.disabled=false; });
};
document.getElementById('dustOn').onchange=e=>{
  const cb=e.target; cb.disabled=true;
  fetch('/api/dust',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({enabled:cb.checked})})
    .then(r=>r.json())
    .then(j=>{ toast(j.ok?('auto empty '+(cb.checked?'on':'off')):(j.error||'failed')); loadDust(); })
    .catch(()=>toast('error'))
    .finally(()=>{ cb.disabled=false; });
};

// --- mop wash frequency. The interval unit follows the trigger, so the label
// changes with the select rather than after the save.
function washUnitLabel(){
  const m=+document.getElementById('washMode').value;
  set('washUnit', m===2?'m²':(m===1?'min':'rooms'));
}
document.getElementById('washMode').onchange=washUnitLabel;
function loadWash(){
  fetch('/api/wash').then(r=>r.json()).then(j=>{
    if(!j.ok){ set('washState', j.error||'unavailable'); return; }
    const w=j.wash;
    set('washState', (j.mode_name||'')+' · every '+w.interval);
    set('washMission', w.in_mission?'washes mid-run':'only at the end');
    document.getElementById('washMode').value=String(w.mode);
    document.getElementById('washInterval').value=w.interval;
    document.getElementById('washDur').value=String(w.duration);
    washUnitLabel();
  }).catch(()=>set('washState','error'));
}
document.getElementById('washSave').onclick=()=>{
  const b=document.getElementById('washSave'); b.disabled=true;
  fetch('/api/wash',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({mode:+document.getElementById('washMode').value,
                         interval:+document.getElementById('washInterval').value,
                         duration:+document.getElementById('washDur').value})})
    .then(r=>r.json())
    .then(j=>{ toast(j.ok?'wash frequency set':(j.error||'failed'), j.ok?'ok':'bad'); loadWash(); })
    .catch(()=>toast('error','bad'))
    .finally(()=>{ b.disabled=false; });
};

// --- mop dry. cfg 0 is off and cannot be encoded, so the select offers two.
function loadDry(){
  fetch('/api/dry').then(r=>r.json()).then(j=>{
    if(!j.ok){ set('dryState', j.error||'unavailable'); return; }
    set('dryState', (j.dry.enabled?'on':'off')+' · '+(j.duration_name||''));
    document.getElementById('dryDur').value=String(j.dry.duration);
  }).catch(()=>set('dryState','error'));
  // work 4 is a dry in progress and it is what refuses an empty, so it belongs
  // next to the setting rather than three pages away.
  fetch('/api/station').then(r=>r.json()).then(j=>{
    if(!j.ok){ set('dryDock', j.error||'-'); return; }
    const w=j.station.work;
    set('dryDock', w===4?'drying now':(w===8?'emptying':(w===0?'idle':'work '+w)));
  }).catch(()=>set('dryDock','-'));
}
document.getElementById('dryStop').onclick=()=>{
  const b=document.getElementById('dryStop'); b.disabled=true;
  fetch('/api/station?action=stopdry',{method:'POST'})
    .then(r=>r.json())
    // peripherals re-issues DRY within the second, so a green toast here is not
    // a promise that the dock is free. The status line above is the answer.
    .then(j=>{ toast(j.ok?'stop sent':(j.error||'failed'), j.ok?'ok':'bad'); loadDry(); })
    .catch(()=>toast('error','bad'))
    .finally(()=>{ b.disabled=false; });
};
document.getElementById('drySave').onclick=()=>{
  const b=document.getElementById('drySave'); b.disabled=true;
  fetch('/api/dry',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({cfg:+document.getElementById('dryCfg').value,
                         duration:+document.getElementById('dryDur').value})})
    .then(r=>r.json())
    .then(j=>{ toast(j.ok?'dry setting saved':(j.error||'failed'), j.ok?'ok':'bad'); loadDry(); })
    .catch(()=>toast('error','bad'))
    .finally(()=>{ b.disabled=false; });
};

// --- off-peak charging window.
function loadPeak(){
  fetch('/api/peakcharge').then(r=>r.json()).then(j=>{
    if(!j.ok){ set('peakState', j.error||'unavailable'); return; }
    const p=j.peak_charge;
    set('peakState', (p.enabled?'on':'off')+' · '+(j.window||''));
    document.getElementById('peakOn').checked=!!p.enabled;
    document.getElementById('peakFrom').value=hhmm({hour:p.start_hour,minute:p.start_minute});
    document.getElementById('peakTo').value=hhmm({hour:p.end_hour,minute:p.end_minute});
  }).catch(()=>set('peakState','error'));
}
document.getElementById('peakSave').onclick=()=>{
  const b=document.getElementById('peakSave'); b.disabled=true;
  const f=(document.getElementById('peakFrom').value||'00:00').split(':');
  const t=(document.getElementById('peakTo').value||'00:00').split(':');
  fetch('/api/peakcharge',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({enabled:document.getElementById('peakOn').checked,
                         start_hour:+f[0], start_minute:+f[1],
                         end_hour:+t[0], end_minute:+t[1]})})
    .then(r=>r.json())
    .then(j=>{ toast(j.ok?'charging window set':(j.error||'failed'), j.ok?'ok':'bad'); loadPeak(); })
    .catch(()=>toast('error','bad'))
    .finally(()=>{ b.disabled=false; });
};

// --- spot clean. Its own endpoint, not /api/command: the method carries a
// param and handleCommand sends none.
document.getElementById('spotGo').onclick=()=>{
  const b=document.getElementById('spotGo'); b.disabled=true;
  set('spotState','starting');
  fetch('/api/spotclean',{method:'POST'})
    .then(r=>r.json())
    .then(j=>{ toast(j.ok?('spot clean via '+(j.bus||'?')):(j.error||'failed'), j.ok?'ok':'bad');
               set('spotState', j.ok?'sent':'failed'); })
    .catch(()=>{ toast('error','bad'); set('spotState','error'); })
    .finally(()=>{ b.disabled=false; });
};

// --- voice volume.
function loadVolume(){
  fetch('/api/volume').then(r=>r.json()).then(j=>{
    if(j.volume===null||j.volume===undefined){ set('volState', j.note||'unreadable'); return; }
    set('volState', j.volume===0?'silent':j.volume+' of 100');
    document.getElementById('volRange').value=j.volume;
    set('volNum', String(j.volume));
  }).catch(()=>set('volState','error'));
}
document.getElementById('volRange').oninput=e=>set('volNum', e.target.value);
document.getElementById('volSave').onclick=()=>{
  const b=document.getElementById('volSave');
  b.disabled=true;
  fetch('/api/volume',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({volume:+document.getElementById('volRange').value})})
    .then(r=>r.json())
    .then(j=>{ toast(j.ok?'volume set':(j.error||'failed')); loadVolume(); })
    .catch(()=>toast('error'))
    .finally(()=>{ b.disabled=false; });
};

// --- the switch block. One POST per switch, carrying only that switch: the
// robot merges what it is sent, and sending the whole block back would write
// every switch on every click including the ones that cannot be read.
function loadUni(){
  fetch('/api/unisettings').then(r=>r.json()).then(j=>{
    const box=document.getElementById('uniList'); box.innerHTML='';
    if(!j.ok){ set('uniState', j.error||'unavailable'); return; }
    let known=0;
    j.fields.forEach(f=>{
      const cur=j.settings[f.name];
      if(cur!==null&&cur!==undefined) known++;
      const row=document.createElement('div'); row.className='row';
      // A switch the robot does not store is not a switch with an unknown
      // value. It was set on hardware and nothing changed anywhere in
      // user_config.yaml, so it gets said plainly instead of a live-looking box.
      const dead=f.mapped===false;
      if(dead) row.style.opacity='.5';
      row.innerHTML='<span class="k">'+f.name.replace(/_sw$/,'').replace(/_/g,' ')
        +'<br><span style="font-size:.72rem;opacity:.6">'+f.desc+'</span></span>'
        +'<span class="v"><label class="lbl"><input type="checkbox" data-sw="'+f.name+'"'
        +(cur===true?' checked':'')+(dead?' disabled':'')+'> '
        +(dead?'no readback':'on')+'</label></span>';
      box.appendChild(row);
    });
    const dead=j.fields.filter(f=>f.mapped===false).length;
    set('uniState', known+' of '+j.fields.length+' readable'
      +(dead?', '+dead+' write-only':''));
    const dust=j.settings.dust_full_remind;
    if(dust!==null&&dust!==undefined) document.getElementById('uniDust').value=dust;
    box.querySelectorAll('input[data-sw]').forEach(cb=>{
      cb.onchange=()=>{
        const body={switches:{}}; body.switches[cb.dataset.sw]=cb.checked;
        cb.disabled=true;
        fetch('/api/unisettings',{method:'POST',headers:{'Content-Type':'application/json'},
          body:JSON.stringify(body)})
          .then(r=>r.json())
          .then(j=>{ toast(j.ok?(cb.dataset.sw+(cb.checked?' on':' off')):(j.error||'failed')); loadUni(); })
          .catch(()=>{ toast('error'); cb.disabled=false; });
      };
    });
  }).catch(()=>set('uniState','error'));
}
document.getElementById('uniDustSave').onclick=()=>{
  const b=document.getElementById('uniDustSave');
  b.disabled=true;
  fetch('/api/unisettings',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({dust_full_remind:+document.getElementById('uniDust').value})})
    .then(r=>r.json())
    .then(j=>{ toast(j.ok?'reminder set':(j.error||'failed')); loadUni(); })
    .catch(()=>toast('error'))
    .finally(()=>{ b.disabled=false; });
};

// --- consumables. Read-only on purpose; see consumables.go.
function loadConsumables(){
  fetch('/api/consumables').then(r=>r.json()).then(j=>{
    const box=document.getElementById('conList'); box.innerHTML='';
    if(!j.ok){ set('conState', j.error||'unavailable'); return; }
    let low=0;
    j.parts.forEach(p=>{
      if(p.attention) low++;
      const row=document.createElement('div'); row.className='row';
      let right='-';
      if(p.percent_left!==null&&p.percent_left!==undefined){
        const hrs=Math.round(p.left_minutes/60);
        right=p.percent_left+'% left · '+hrs+' h';
      } else if(p.total_minutes!==null&&p.total_minutes!==undefined){
        right=p.total_minutes+' min · no pair';
      } else if(p.left_minutes!==null&&p.left_minutes!==undefined){
        right=p.left_minutes+' min · no pair';
      }
      row.innerHTML='<span class="k">'+p.name+' <span class="tag">'+p.where+'</span></span>'
        +'<span class="v'+(p.attention?' hit':'')+'">'+right+'</span>';
      box.appendChild(row);
      (p.notes||[]).forEach(n=>{
        const d=document.createElement('div'); d.className='note'; d.textContent=n; box.appendChild(d);
      });
    });
    set('conState', low?(low+' need attention'):'all above 10%');
    const t=j.totals||{};
    document.getElementById('conTotals').textContent=
      'Lifetime: '+(t.area_m2||0)+' m2 over '+(t.cleans||0)+' cleans, '
      +Math.round((t.duration_minutes||0)/60)+' h. '
      +(t.area_m2_since_bag_change!=null ? t.area_m2_since_bag_change+' m2 since the bag was changed.' : '');
  }).catch(()=>set('conState','error'));
}

// --- clock. The robot has no NTP and no cloud to ask, so the browser is the
// time source: whatever device is driving the robot already knows the time.
function loadClock(){
  fetch('/api/time').then(r=>r.json()).then(j=>{
    if(!j.ok){ set('clkState','unavailable'); return; }
    const sys=new Date(j.epoch_ms), skew=Math.round((Date.now()-j.epoch_ms)/1000);
    set('clkState', sys.toLocaleString()+(j.plausible?'':' \u00b7 UNSET')
      +(Math.abs(skew)>60?' \u00b7 '+skew+'s off this device':''));
    set('clkDrift', j.rtc_drift_seconds==null ? 'no RTC read'
      : j.rtc_drift_seconds+'s vs system');
  }).catch(()=>set('clkState','error'));
}
document.getElementById('clkSync').onclick=()=>{
  const b=document.getElementById('clkSync'); b.disabled=true;
  fetch('/api/time?epoch_ms='+Date.now(),{method:'POST'})
    .then(r=>r.json())
    .then(j=>{ toast(j.ok?'clock set, RTC written':(j.error||'failed')); loadClock(); })
    .catch(()=>toast('error'))
    .finally(()=>{ b.disabled=false; });
};

// --- per-room settings. Sent as one batch on Save rather than per control:
// a map edit is a heavier operation than a clean-parameter change, and firing
// one per dropdown while somebody works down eight rooms is unkind to it.
let roomSetNames=null, roomSetView=null;
function loadRoomSettings(){
  fetch('/api/roomsettings').then(r=>r.json()).then(j=>{
    const box=document.getElementById('roomSetList'); box.innerHTML='';
    if(!j.ok){ set('roomSetState', j.error||'unavailable'); return; }
    roomSetNames=j.names; roomSetView=j.view;
    set('roomSetState', j.view.map_name+' \u00b7 '+j.view.rooms.length+' rooms');
    document.getElementById('roomCustom').checked=!!j.view.custom_enable;
    j.view.rooms.forEach(rm=>{
      const p=rm.params||{clean_type:0,fan:0,mop_water:0,clean_extent:0,clean_times:1};
      const d=document.createElement('div'); d.className='card';
      d.style.margin='8px 0'; d.style.padding='8px 10px';
      const sel=(key,cur)=>'<select data-room="'+rm.ID+'" data-key="'+key+'">'
        +(j.names[key]||[]).map((n,i)=>'<option value="'+i+'"'+(i===cur?' selected':'')+'>'
          +n.replace(/_/g,' ')+'</option>').join('')+'</select>';
      d.innerHTML='<div class="row"><span class="k">'+rm.name+'</span>'
        +'<span class="v">order <input type="number" min="0" max="8" style="width:3.5em"'
        +' data-room="'+rm.ID+'" data-key="order" value="'+(rm.order||0)+'"></span></div>'
        +'<div class="row"><span class="k">Mode</span><span class="v">'+sel('clean_type',p.clean_type)+'</span></div>'
        +'<div class="row"><span class="k">Suction</span><span class="v">'+sel('fan',p.fan)+'</span></div>'
        +'<div class="row"><span class="k">Water</span><span class="v">'+sel('mop_water',p.mop_water)+'</span></div>'
        +'<div class="row"><span class="k">Repeats</span><span class="v">'
        +'<input type="number" min="1" max="3" style="width:3.5em" data-room="'+rm.ID
        +'" data-key="clean_times" value="'+(p.clean_times||1)+'"></span></div>';
      box.appendChild(d);
    });
    // Rename is its own edit - a RoomDesc, not a RoomsCustom - so it commits on
    // blur rather than waiting for the Save button, which sends parameters.
    // Merge picker, built from the same room list.
    const mb=document.getElementById('mergeList'); mb.innerHTML='';
    j.view.rooms.forEach(rm=>{
      const row=document.createElement('div'); row.className='row';
      row.innerHTML='<span class="k"><label class="lbl"><input type="checkbox" class="mrg" value="'
        +rm.ID+'"> '+rm.name+'</label></span><span class="v">#'+rm.ID+'</span>';
      mb.appendChild(row);
    });
    set('mergeState','pick two or more');
    box.querySelectorAll('input.rname').forEach(el=>{
      const was=el.value;
      el.onblur=()=>{
        const name=el.value.trim();
        if(!name||name===was){ el.value=was; return; }
        fetch('/api/roomrename',{method:'POST',headers:{'Content-Type':'application/json'},
          body:JSON.stringify({id:+el.dataset.room,name:name})})
          .then(r=>r.json())
          .then(j=>{ toast(j.ok?'renamed':(j.error||'failed')); loadRoomSettings(); })
          .catch(()=>toast('error'));
      };
      el.onkeydown=e=>{ if(e.key==='Enter') el.blur(); };
    });
  }).catch(()=>set('roomSetState','error'));
}
function mapRevert(action,label){
  return ()=>{
    fetch('/api/maprevert?action='+action,{method:'POST'})
      .then(r=>r.json())
      .then(j=>{ toast(j.ok?label:(j.error||'failed')); loadRoomSettings(); })
      .catch(()=>toast('error'));
  };
}
document.getElementById('mapUndo').onclick=mapRevert('undo','undid last edit');
document.getElementById('mapDone').onclick=mapRevert('done','edit session closed');
document.getElementById('mergeGo').onclick=()=>{
  const ids=[...document.querySelectorAll('input.mrg:checked')].map(e=>+e.value);
  if(ids.length<2){ toast('pick at least two rooms'); return; }
  if(!confirm('Merge rooms '+ids.join(', ')+'? This rewrites the room map.')) return;
  const b=document.getElementById('mergeGo'); b.disabled=true;
  set('mergeState','merging...');
  fetch('/api/roomsmerge',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({rooms:ids})})
    .then(r=>r.json())
    .then(j=>{ toast(j.ok?'merged':(j.error||'failed'));
               set('mergeState', j.ok?'merged':(j.error||'failed')); loadRoomSettings(); })
    .catch(()=>{ toast('error'); set('mergeState','error'); })
    .finally(()=>{ b.disabled=false; });
};
document.getElementById('roomSetSave').onclick=()=>{
  if(!roomSetView) return;
  const by={};
  roomSetView.rooms.forEach(rm=>{
    const p=rm.params||{};
    by[rm.ID]={ID:rm.ID,order:rm.order||0,params:{
      clean_type:p.clean_type||0, fan:p.fan||0, mop_water:p.mop_water||0,
      corner_clean:p.corner_clean||0, clean_extent:p.clean_extent||0,
      clean_times:p.clean_times||1}};
  });
  document.querySelectorAll('#roomSetList [data-room]').forEach(el=>{
    const r=by[+el.dataset.room]; if(!r) return;
    if(el.dataset.key==='order') r.order=+el.value;
    else r.params[el.dataset.key]=+el.value;
  });
  const btn=document.getElementById('roomSetSave'); btn.disabled=true;
  fetch('/api/roomsettings',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({rooms:Object.values(by),
      custom_enable:document.getElementById('roomCustom').checked})})
    .then(r=>r.json())
    .then(j=>{ toast(j.ok?'room settings saved':(j.error||'failed')); loadRoomSettings(); })
    .catch(()=>toast('error'))
    .finally(()=>{ btn.disabled=false; });
};

// --- drawing keep-out zones on the map.
//
// Zones are held in world metres, not pixels, so they survive a map redraw and
// go to the robot in the units it wants. The pointer gives client pixels, which
// go through the image box, then the flip, then the map frame - the same chain
// the dock marker validated, run backwards.
let zoneDraft=null, zoneList=null, zoneFrame=null, zoneDrag=null;

function zoneToWorld(ev){
  const img=document.getElementById('map'), r=img.getBoundingClientRect();
  if(!zoneFrame||!r.width||!r.height) return null;
  // Clamped because the pointer is captured: a drag that ends past the edge of
  // the image should give a zone that stops at the edge, not one off the map.
  const clamp=(v,hi)=>v<0?0:(v>hi?hi:v);
  let px=clamp((ev.clientX-r.left)/r.width*zoneFrame.width, zoneFrame.width);
  let py=clamp((ev.clientY-r.top)/r.height*zoneFrame.height, zoneFrame.height);
  // Undo the render flip before converting - the PNG is mirrored, the
  // coordinates are not.
  if(mapFlipMode==='h'||mapFlipMode==='both') px=zoneFrame.width-px;
  if(mapFlipMode==='v'||mapFlipMode==='both') py=zoneFrame.height-py;
  return {x: px*zoneFrame.resolution+zoneFrame.origin_x,
          y: py*zoneFrame.resolution+zoneFrame.origin_y};
}

// zoneToSvg is zoneToWorld run forwards, for drawing the drag as it happens.
// Same transform as the overlay, which the dock marker validated.
function zoneToSvg(p){
  if(!zoneFrame) return null;
  let px=(p.x-zoneFrame.origin_x)/zoneFrame.resolution;
  let py=(p.y-zoneFrame.origin_y)/zoneFrame.resolution;
  if(mapFlipMode==='h'||mapFlipMode==='both') px=zoneFrame.width-px;
  if(mapFlipMode==='v'||mapFlipMode==='both') py=zoneFrame.height-py;
  return {x:px, y:py};
}

// zonePreview draws the rubber band. Without it the drag has no feedback at
// all, so a gesture that did nothing and a gesture that was stolen by the
// browser looked exactly alike.
function zonePreview(a,b){
  const svg=document.getElementById('mapOv');
  const old=document.getElementById('zonePrev');
  if(old) old.remove();
  if(!a||!b) return;
  const p=zoneToSvg(a), q=zoneToSvg(b);
  if(!p||!q) return;
  const el=zoneDrag&&zoneDrag.kind==='wall'||zoneDrag&&zoneDrag.kind==='split'
    ? '<line id="zonePrev" x1="'+p.x.toFixed(1)+'" y1="'+p.y.toFixed(1)
      +'" x2="'+q.x.toFixed(1)+'" y2="'+q.y.toFixed(1)
      +'" stroke="#ff5c5c" stroke-width="2" stroke-dasharray="4 3"/>'
    : '<rect id="zonePrev" x="'+Math.min(p.x,q.x).toFixed(1)+'" y="'+Math.min(p.y,q.y).toFixed(1)
      +'" width="'+Math.abs(q.x-p.x).toFixed(1)+'" height="'+Math.abs(q.y-p.y).toFixed(1)
      +'" fill="#ff5c5c" fill-opacity="0.2" stroke="#ff5c5c" stroke-width="1.2"'
      +' stroke-dasharray="4 3"/>';
  svg.insertAdjacentHTML('beforeend', el);
}

function zoneMapEvents(){
  const img=document.getElementById('map');
  img.style.touchAction='none';
  // Belt and braces with draggable="false" and the CSS: Safari still starts a
  // drag from an img on some paths, and this is the event that would do it.
  img.addEventListener('dragstart',ev=>ev.preventDefault());
  img.addEventListener('pointerdown',ev=>{
    const kind=document.getElementById('zoneKind').value;
    if(!kind) return;
    ev.preventDefault();
    // Both of these used to fail silently, which looks the same as a broken
    // drag. The frame comes from the overlay, and a layer with its own frame
    // would put the zone somewhere else entirely.
    if(!mapLayerMain){ toast('switch to Rooms or Occupancy to draw','bad'); return; }
    if(!zoneFrame){ toast('turn the overlay on first','bad'); return; }
    // Capture, so a drag that finishes off the edge of the image still counts.
    // Without it the pointerup lands on the page and the zone is lost.
    try{ img.setPointerCapture(ev.pointerId); }catch(e){}
    zoneDrag={kind:kind, a:zoneToWorld(ev), id:ev.pointerId};
  });
  img.addEventListener('pointermove',ev=>{
    if(!zoneDrag) return;
    ev.preventDefault();
    zonePreview(zoneDrag.a, zoneToWorld(ev));
  });
  img.addEventListener('pointercancel',()=>{ zoneDrag=null; zonePreview(null,null); });
  img.addEventListener('pointerup',ev=>{
    if(!zoneDrag) return;
    try{ img.releasePointerCapture(zoneDrag.id); }catch(e){}
    zonePreview(null,null);
    const b=zoneToWorld(ev), a=zoneDrag.a, kind=zoneDrag.kind;
    zoneDrag=null;
    if(!a||!b) return;
    // A tap is not a zone. 15 cm is under the robot's own width.
    if(Math.abs(a.x-b.x)<0.15&&Math.abs(a.y-b.y)<0.15){ toast('drag to size the zone'); return; }
    if(!zoneList) zoneList=[];
    if(kind==='split'){
      // A split is a map edit in its own right, not a zone, so it goes
      // immediately rather than waiting for Save zones.
      const room=+document.getElementById('splitRoom').value;
      if(!room){ toast('pick a room to split'); return; }
      if(!confirm('Split room '+room+' along this line?')) return;
      fetch('/api/roomdivide',{method:'POST',headers:{'Content-Type':'application/json'},
        body:JSON.stringify({room:room,points:[a,b]})})
        .then(r=>r.json())
        .then(j=>{ toast(j.ok?'room split':(j.error||'failed'));
                   if(j.ok){ roomSetView=j.view; drawMap(); } })
        .catch(()=>toast('error'));
      return;
    }
    if(kind==='wall') zoneList.push({kind:'wall',points:[a,b]});
    else zoneList.push({kind:kind,points:[
      {x:a.x,y:a.y},{x:b.x,y:a.y},{x:b.x,y:b.y},{x:a.x,y:b.y}]});
    zoneDraft=true;
    drawOverlay();
    toast(kind+' added - press Save zones');
  });
}

// The split picker only makes sense in split mode, and needs the room list.
document.getElementById('zoneKind').addEventListener('change',()=>{
  const split=document.getElementById('zoneKind').value==='split';
  document.getElementById('splitRow').hidden=!split;
  if(!split) return;
  const sel=document.getElementById('splitRoom');
  const fill=v=>{ sel.innerHTML=(v.rooms||[]).map(r=>'<option value="'+r.id+'">'+r.name+'</option>').join(''); };
  if(roomSetView) fill(roomSetView);
  else fetch('/api/roomsettings').then(r=>r.json())
        .then(j=>{ if(j.ok){ roomSetView=j.view; fill(j.view); } }).catch(()=>{});
});
document.getElementById('zoneUndo').onclick=()=>{
  if(!zoneList||!zoneList.length){ toast('nothing to remove'); return; }
  zoneList.pop(); zoneDraft=true; drawOverlay();
};
document.getElementById('zoneSave').onclick=()=>{
  if(!zoneDraft){ toast('no changes'); return; }
  const b=document.getElementById('zoneSave'); b.disabled=true;
  fetch('/api/zones',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({zones:zoneList||[]})})
    .then(r=>r.json())
    .then(j=>{ toast(j.ok?'zones saved':(j.error||'failed'));
               zoneDraft=false; zoneList=null; drawOverlay(); })
    .catch(()=>toast('error'))
    .finally(()=>{ b.disabled=false; });
};

// --- manual mode page: camera, forward lidar arc, contact sensors, drive.
//
// The lidar view is the forward arc only. A full sweep is the wrong picture
// when steering by hand - what matters is what is in front, at the scale of the
// next second of driving - so this clips to the upper half and to 3 m.
let manOn=false, manTimer=null, manViewOnly=false;
const MAN_RANGE=3.0;

function manLidar(){
  fetch('/api/lidar').then(r=>r.json()).then(j=>{
    const svg=document.getElementById('mlidar');
    if(!j.ok||!j.scan){ svg.innerHTML=''; return; }
    const sc=j.scan, n=sc.ranges.length;
    let pts='', near=99;
    for(let i=0;i<n;i++){
      const r=sc.ranges[i];
      if(!isFinite(r)||r<sc.range_min||r>MAN_RANGE) continue;
      const a=(sc.angles&&sc.angles.length===n)?sc.angles[i]:sc.angle_min+i*sc.angle_increment;
      const p=lidProject(a, r/MAN_RANGE);
      if(p[1]>0.02) continue;              // behind the robot
      if(Math.abs(p[0])<0.5 && r<near) near=r;
      pts+='<circle cx="'+p[0].toFixed(4)+'" cy="'+p[1].toFixed(4)+'" r="0.014"/>';
    }
    svg.innerHTML='<path d="M -1 0 A 1 1 0 0 1 1 0" fill="none" stroke="#8884" stroke-width="0.008"/>'
      +'<path d="M -0.5 0 A 0.5 0.5 0 0 1 0.5 0" fill="none" stroke="#8883" stroke-width="0.006"/>'
      +'<g fill="var(--accent)">'+pts+'</g>'
      +'<circle cx="0" cy="0" r="0.035" fill="var(--move)"/>';
    set('mAhead', near<90 ? near.toFixed(2)+' m ahead' : 'clear to '+MAN_RANGE+' m');
  }).catch(()=>{});
}

function manSensors(){
  fetch('/api/sensors').then(r=>r.json()).then(j=>{
    if(!j.ok){ set('mBump','no report'); return; }
    const s=j.sensors, hit=[];
    if(s.bump_left) hit.push('BUMP L');
    if(s.bump_right) hit.push('BUMP R');
    if(s.cliff) hit.push('CLIFF '+s.ground_seen+'/'+s.ground_sensors);
    if(s.radar_left) hit.push('radar L');
    if(s.radar_mid) hit.push('radar M');
    if(s.radar_right) hit.push('radar R');
    if(s.front_wall) hit.push('wall ahead');
    if(s.carpet) hit.push('carpet');
    const el=document.getElementById('mBump');
    el.className = hit.length ? 'v hit' : 'v';
    el.textContent = hit.length ? hit.join(' \u00b7 ') : 'clear';
  }).catch(()=>{});
}

function manTick(){
  document.getElementById('mcam').src='/api/camera.jpg?scale=3&q=70&t='+Date.now();
  manLidar(); manSensors();
}

function manualEnter(on,force){
  fetch('/api/manual?on='+(on?1:0)+(force?'&force=1':''),{method:'POST'})
    .then(r=>r.json())
    .then(j=>{
      // Mid-run the server refuses rather than reconfiguring the clean, so the
      // override is a decision the user makes, not something a tab switch does
      // on their behalf.
      if(j.busy){
        if(confirm('Cleaning in progress. Really override and start manual control?')){
          manualEnter(true,true);
          return;
        }
        // Declined. manOn stays false on purpose: leaving the page must not
        // then send the stop that was just refused.
        manOn=false; manViewOnly=true;
        set('manState','view only \u00b7 cleaning in progress');
        return;
      }
      manViewOnly=false;
      set('manState', j.ok ? (on?'on \u00b7 sweep only':'off \u00b7 '+j.clean_type)
                           : (j.error||'failed'));
    })
    .catch(()=>set('manState','error'));
}

function manualPage(active){
  clearInterval(manTimer); manTimer=null;
  if(active===manOn) return;
  manOn=active;
  manualEnter(active);
  // The camera, lidar and sensor reads are read-only shared-memory attaches, so
  // they keep ticking even when the override was declined. Watching costs the
  // robot nothing; only entering the mode does.
  if(active){ manTick(); manTimer=setInterval(()=>{ if(current==='manual') manTick(); },700); }
  else { clearInterval(driveTimer); driveTimer=null; driveDir=null;
         clearTimeout(parkTimer); parkTimer=null; driving=false; manViewOnly=false; }
}
document.getElementById('mLight').onchange=()=>{
  const on=document.getElementById('mLight').checked;
  fetch('/api/camera/light?level='+(on?100:0),{method:'POST'}).catch(()=>toast('light error'));
};

// --- manual drive.
//
// Two things this has to get right, both learned the hard way on hardware.
//
// A held button must stay held. pointerleave fires the moment the cursor drifts
// a pixel off the button, and treating that as a release made the robot stop by
// itself mid-drive. setPointerCapture keeps the events coming to the button
// until the actual pointerup.
//
// And a release must NOT leave remote mode. BRAKE stops the wheels and stays in
// the mode by design; ending the task costs about two seconds to re-enter on
// the next press, which reads exactly like the robot stopping on its own. So
// release brakes, and parking - which is what retracts the mops - runs off an
// idle timer instead.
const PARK_IDLE_MS=4000;
let driveTimer=null, driveDir=null, parkTimer=null, driving=false;

// Both D-pads - Control and Camera - drive the same state, so a press on
// either is the same press. driveSay updates whichever readouts are on screen.
function driveSay(msg){ ['driveState','driveState2'].forEach(id=>set(id,msg)); }
function driveSend(dir,park){
  // Watching a run is free, steering into it is not. Without the mode the
  // robot never entered, a direction still goes out as FunctionalCtrl and
  // fights the clean it is in the middle of.
  if(manViewOnly){ driveSay('view only · not in manual mode'); return Promise.resolve({ok:false}); }
  return fetch('/api/move?dir='+dir+(park?'&park=1':''),{method:'POST'})
    .then(r=>r.json())
    .then(j=>{
      if(!j.ok){ driveSay(j.error||'failed'); return j; }
      driveSay(park?'parked, mops up':(dir==='brake'?'idle':dir+' \u00b7 '+j.bus));
      return j;
    })
    .catch(()=>driveSay('error'));
}
// One press, one command. The vendor app drives this way and the capture on
// 2026-08-07 shows why: 19 direction frames, no brake and no repeat.
function driveStart(dir){
  clearTimeout(parkTimer); parkTimer=null;
  driveDir=dir; driving=true;
  driveSend(dir);
  parkTimer=setTimeout(()=>{ if(driving){ driving=false; driveSend('brake',true); } },PARK_IDLE_MS);
}
function driveRelease(){
  clearInterval(driveTimer); driveTimer=null;
}
function drivePark(){
  clearInterval(driveTimer); driveTimer=null; driveDir=null;
  clearTimeout(parkTimer); parkTimer=null; driving=false;
  driveSend('brake',true);
}
document.querySelectorAll('.dpad button').forEach(b=>{
  const dir=b.dataset.dir;
  if(dir==='brake'){ b.onclick=drivePark; return; }
  b.addEventListener('pointerdown',e=>{
    e.preventDefault();
    driveStart(dir);
  });
  b.addEventListener('contextmenu',e=>e.preventDefault());
});
// Losing the page while driving must not leave it driving, and must still park.
// Leaving the page while it is driving must stop it, not just stop the UI.
window.addEventListener('blur',()=>{ if(driveDir){ driveDir=null; driveSend('brake'); } });
document.addEventListener('visibilitychange',()=>{
  if(document.hidden&&driveDir){ driveDir=null; driveSend('brake'); } });

// --- bus. The mds registry, one record at a time.
//
// The dropdown marks a channel live or not with a filled or hollow dot. Not
// live is a real answer: most of the registry is only published while the thing
// it describes is running, so an empty segment at idle is expected rather than
// broken.
let busTimer=null, busLoaded=false;
function busRender(fields,indent){
  return (fields||[]).map(f=>{
    const pad='  '.repeat(indent);
    if(f.type==='message') return pad+'f'+f.field+'  message\n'+busRender(f.fields,indent+1);
    let v=f.value;
    if(f.type==='packed float32') v='['+v.count+'] '+(v.first||[]).join(' ')+' ...';
    else if(f.type==='string') v=JSON.stringify(v);
    return pad+'f'+f.field+'  '+f.type+'  '+v;
  }).join('\n');
}
function busRead(){
  const c=document.getElementById('busChan').value;
  if(!c) return;
  fetch('/api/mds?c='+encodeURIComponent(c)).then(r=>r.json()).then(j=>{
    const out=document.getElementById('busOut');
    if(!j.ok){ set('busState', j.error||'failed'); out.textContent=''; return; }
    set('busState', j.bytes+' bytes');
    out.textContent=busRender(j.fields,0);
  }).catch(()=>set('busState','error'));
}
function busPage(active){
  clearInterval(busTimer); busTimer=null;
  if(!active) return;
  if(!busLoaded){
    busLoaded=true;
    fetch('/api/mds').then(r=>r.json()).then(j=>{
      const sel=document.getElementById('busChan');
      (j.channels||[]).forEach(c=>{
        const o=document.createElement('option');
        o.value=c.name;
        o.textContent=(c.live?'● ':'○ ')+c.name.replace('eros.proto.msg.','');
        sel.appendChild(o);
      });
      busRead();
    }).catch(()=>set('busState','channel list failed'));
  } else busRead();
  if(document.getElementById('busLive').checked){
    busTimer=setInterval(()=>{ if(current==='bus') busRead(); },1000);
  }
}
document.getElementById('busRead').onclick=busRead;
document.getElementById('busChan').onchange=busRead;
document.getElementById('busLive').onchange=()=>busPage(current==='bus');

// --- lidar. Polar sweep, drawn as SVG so the robot encodes nothing.
//
// Each point carries its own angle (field 13 of the scan), so the sweep is
// drawn from those rather than from angle_min plus i*increment. The two agree
// on this unit, but the per-point array is what the robot itself uses and it
// costs nothing to prefer it.
const lidLive=document.getElementById('lidLive');
let lidTimer=null, lidBusy=false;

// Measured against the room, not derived from the mounting transform.
// Why the two disagree: docs/buses.md.
const lidRot=150, lidMirror=true;
function lidProject(a, r){
  // +angle is counter-clockwise on screen because y is negated below, so a
  // clockwise rotation of the picture subtracts.
  let t=a-lidRot*Math.PI/180;
  if(lidMirror) t=Math.PI-t;
  return [r*Math.cos(t), -r*Math.sin(t)];
}
function loadLidar(){
  if(lidBusy) return;
  lidBusy=true;
  fetch('/api/lidar').then(r=>r.json()).then(j=>{
    const svg=document.getElementById('lidar');
    if(!j.ok){ set('lidInfo', j.error||'unavailable'); svg.innerHTML=''; return; }
    const sc=j.scan, n=sc.ranges.length;
    let hits=0, pts='';
    for(let i=0;i<n;i++){
      const r=sc.ranges[i];
      // A zero range is "no return", not a wall at the origin.
      if(!isFinite(r)||r<sc.range_min||r>sc.range_max) continue;
      hits++;
      const raw=(sc.angles&&sc.angles.length===n)?sc.angles[i]:sc.angle_min+i*sc.angle_increment;
      const p=lidProject(raw, r/sc.range_max);
      pts+='<circle cx="'+p[0].toFixed(4)+'" cy="'+p[1].toFixed(4)+'" r="0.012"/>';
    }
    set('lidInfo', hits+'/'+n+' returns \u00b7 '+sc.range_max.toFixed(1)+' m max \u00b7 '
      +(sc.age<2?'live':sc.age.toFixed(0)+'s old'));
    // Heading marker, fixed at 12 o'clock. It is the reference the scan is
    // rotated onto; computing it would point at the sensor's zero and mislead.
    const h=[0,-0.86];
    svg.innerHTML='<circle cx="0" cy="0" r="0.985" fill="none" stroke="#8884" stroke-width="0.006"/>'
      +'<circle cx="0" cy="0" r="0.49" fill="none" stroke="#8883" stroke-width="0.004"/>'
      +'<g fill="var(--accent)">'+pts+'</g>'
      +'<line x1="0" y1="0" x2="'+h[0].toFixed(4)+'" y2="'+h[1].toFixed(4)
      +'" stroke="var(--move)" stroke-width="0.014"/>'
      +'<circle cx="0" cy="0" r="0.03" fill="var(--move)"/>';
  }).catch(e=>set('lidInfo','error')).finally(()=>{ lidBusy=false; });
}
function lidarTick(){
  clearInterval(lidTimer); lidTimer=null;
  if(lidLive.checked) lidTimer=setInterval(()=>{ if(current==='lidar') loadLidar(); },1000);
}
lidLive.onchange=lidarTick;
lidarTick();
document.getElementById('lidOff').onclick=()=>{
  lidLive.checked=false; lidarTick();
  fetch('/api/lidar?power=off').then(()=>set('lidInfo','spun down'));
};

// --- map orientation. The stored PNG is mirrored relative to the real home;
// which axis was settled by eye, so the control stays for re-checking.
const FLIPS=[['h','Mirror'],['none','Raw'],['v','Flip V'],['both','Both']];
let mapFlipMode='h';
(function(){
  const wrap=document.getElementById('mapFlip');
  FLIPS.forEach(([k,label],i)=>{
    const b=document.createElement('button');
    b.textContent=label; b.className=i===0?'on':'';
    b.onclick=()=>{ [...wrap.children].forEach(c=>c.className=''); b.className='on';
                    mapFlipMode=k; drawMap(); };
    wrap.appendChild(b);
  });
})();

refresh(); setInterval(refresh,15000);
</script></body></html>`
