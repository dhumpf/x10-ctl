package app

// Home Assistant MQTT autodiscovery. Off unless --ha is given.
//
// Device-based discovery: one retained payload describes every entity at once,
// so the shared device block cannot drift between components.

import (
	"context"
	"encoding/json"
	"fmt"
	"github.com/dhumpf/x10-ctl/internal/client"
	"github.com/dhumpf/x10-ctl/internal/dp"
	"github.com/dhumpf/x10-ctl/internal/mqtt"
	"log"
	"sort"
	"strconv"
	"strings"
	"sync"
	"time"
)

const (
	haPrefix    = "homeassistant"
	haOnline    = "online"
	haOffline   = "offline"
	haSWVersion = "x10-ctl"

	// BatteryInfo charge_state. 0 and 2 are observed; 1 is by elimination.
	chargeStateCharging = 1

	// Long enough to find the robot, short enough not to be a nuisance.
	haLocateBeep = 5 * time.Second

	// Each dock read spawns akostopic, so this is deliberately infrequent.
	haStationEvery = 5 * time.Minute
)

// The nine strings core accepts in supported_features. Anything else makes HA
// reject the whole config.
var haFeatures = []string{
	"start", "stop", "pause", "return_home", "status",
	"locate", "fan_speed", "send_command",
}

// haActivity maps x10-ctl work states onto the six activities HA allows.
// remote_ctrl is deliberately unmapped: the robot is moving but not cleaning,
// and HA has no activity that says so.
var haActivity = map[string]string{
	"cleaning":  "cleaning",
	"docked":    "docked",
	"paused":    "paused",
	"returning": "returning",
	"off_dock":  "idle",
	"transient": "idle",
}

type haBridge struct {
	srv  *server
	bus  *mqtt.ClientBus
	base string
	node string

	mu     sync.Mutex
	last   map[string]string
	paused bool
}

func newHABridge(srv *server, serial string) *haBridge {
	if serial == "" {
		serial = "unknown"
	}
	return &haBridge{
		srv:  srv,
		node: "x10_" + serial,
		base: "x10ctl/" + serial,
		last: map[string]string{},
	}
}

func (h *haBridge) availTopic() string { return h.base + "/availability" }

// attach wires the bridge onto its own broker client, separate from the cloud
// session so the two protocols never share a subscription.
func (h *haBridge) attach(bus *mqtt.ClientBus) {
	h.bus = bus
	bus.Topics = []string{h.base + "/cmd/#", haPrefix + "/status"}
	// Only the will: replacing opts would wipe credentials set by the caller.
	bus.Opts.Will = &mqtt.Will{
		Topic:   h.availTopic(),
		Payload: []byte(haOffline),
		QoS:     1,
		Retain:  true,
	}
	bus.OnMsg = h.onCommand
	bus.OnUp = h.onConnect
}

// onConnect republishes everything. Our broker keeps no retained store, so a
// broker restart would otherwise leave HA with entities it cannot describe.
func (h *haBridge) onConnect() {
	h.mu.Lock()
	h.last = map[string]string{}
	h.mu.Unlock()

	if err := h.publishDiscovery(); err != nil {
		log.Printf("ha: discovery: %v", err)
		return
	}
	if err := h.bus.PublishRetained(h.availTopic(), []byte(haOnline)); err != nil {
		log.Printf("ha: availability: %v", err)
	}
	h.publishState()
	go h.publishStation()
}

// pub sends only what changed. Retained state on an unchanged value would still
// cost a broker round trip for every entity on every tick.
func (h *haBridge) pub(topic, payload string) {
	h.mu.Lock()
	if h.last[topic] == payload {
		h.mu.Unlock()
		return
	}
	h.last[topic] = payload
	h.mu.Unlock()
	if err := h.bus.PublishRetained(topic, []byte(payload)); err != nil {
		log.Printf("ha: publish %s: %v", topic, err)
		h.mu.Lock()
		delete(h.last, topic)
		h.mu.Unlock()
	}
}

func (h *haBridge) device() map[string]any {
	sn := strings.TrimPrefix(h.node, "x10_")
	return map[string]any{
		"ids":  []string{h.node},
		"name": "eufy X10 Pro Omni",
		"mf":   "eufy",
		"mdl":  "T2351",
		"sn":   sn,
		"sw":   haSWVersion,
	}
}

// component is one entry in the device-discovery cmps map.
func (h *haBridge) component(platform, objectID string, extra map[string]any) map[string]any {
	c := map[string]any{
		"p":            platform,
		"unique_id":    h.node + "_" + objectID,
		"avty_t":       h.availTopic(),
		"pl_avail":     haOnline,
		"pl_not_avail": haOffline,
	}
	for k, v := range extra {
		c[k] = v
	}
	return c
}

func (h *haBridge) components() map[string]any {
	cmps := map[string]any{}

	cmps["vacuum"] = h.component("vacuum", "vacuum", map[string]any{
		"name":                         nil,
		"state_topic":                  h.base + "/state",
		"command_topic":                h.base + "/cmd/vacuum",
		"send_command_topic":           h.base + "/cmd/send",
		"set_fan_speed_topic":          h.base + "/cmd/fan_speed",
		"clean_segments_command_topic": h.base + "/cmd/clean_segments",
		"supported_features":           haFeatures,
		"fan_speed_list":               suctionNames,
	})

	// Battery stopped being part of the vacuum entity in HA 2026.8 and has to
	// stand on its own.
	cmps["battery"] = h.component("sensor", "battery", map[string]any{
		"name":         "Battery",
		"state_topic":  h.base + "/battery",
		"dev_cla":      "battery",
		"unit_of_meas": "%",
		"stat_cla":     "measurement",
		"ent_cat":      "diagnostic",
	})

	cmps["charging"] = h.component("binary_sensor", "charging", map[string]any{
		"name":        "Charging",
		"state_topic": h.base + "/charging",
		"dev_cla":     "battery_charging",
		"pl_on":       "ON",
		"pl_off":      "OFF",
		"ent_cat":     "diagnostic",
	})

	cmps["water_tank"] = h.component("sensor", "water_tank", map[string]any{
		"name":         "Clean water tank",
		"state_topic":  h.base + "/water_tank",
		"unit_of_meas": "%",
		"stat_cla":     "measurement",
		"icon":         "mdi:water",
	})

	cmps["water_empty"] = h.component("binary_sensor", "water_empty", map[string]any{
		"name":        "Water tank empty",
		"state_topic": h.base + "/water_empty",
		"dev_cla":     "problem",
		"pl_on":       "ON",
		"pl_off":      "OFF",
	})

	cmps["wastewater"] = h.component("binary_sensor", "wastewater", map[string]any{
		"name":        "Wastewater tank",
		"state_topic": h.base + "/wastewater",
		"dev_cla":     "problem",
		"pl_on":       "ON",
		"pl_off":      "OFF",
	})

	cmps["dust_bag"] = h.component("binary_sensor", "dust_bag", map[string]any{
		"name":        "Dust bag missing",
		"state_topic": h.base + "/dust_bag",
		"dev_cla":     "problem",
		"pl_on":       "ON",
		"pl_off":      "OFF",
	})

	cmps["dock_busy"] = h.component("binary_sensor", "dock_busy", map[string]any{
		"name":        "Dock busy",
		"state_topic": h.base + "/dock_busy",
		"dev_cla":     "running",
		"pl_on":       "ON",
		"pl_off":      "OFF",
		"ent_cat":     "diagnostic",
	})

	cmps["status"] = h.component("sensor", "status", map[string]any{
		"name":        "Status",
		"state_topic": h.base + "/status",
		"ent_cat":     "diagnostic",
	})

	cmps["water_level"] = h.component("select", "water_level", map[string]any{
		"name":          "Water level",
		"state_topic":   h.base + "/water_level",
		"command_topic": h.base + "/cmd/water_level",
		"options":       mopNames,
	})

	cmps["clean_type"] = h.component("select", "clean_type", map[string]any{
		"name":          "Clean type",
		"state_topic":   h.base + "/clean_type",
		"command_topic": h.base + "/cmd/clean_type",
		"options":       typeNames,
	})

	for _, b := range haButtons {
		cmps[b.id] = h.component("button", b.id, map[string]any{
			"name":          b.name,
			"command_topic": h.base + "/cmd/button/" + b.id,
			"ent_cat":       b.category,
		})
	}
	return cmps
}

type haButton struct{ id, name, cmd, category string }

// No wash button: busStationActions carries only dry and dust, so wash falls
// through to the data point and cannot fire while --mqtt is off.
var haButtons = []haButton{
	{"dust", "Empty dust bin", "dust", "config"},
	{"dry", "Dry mop", "dry", "config"},
	{"gowash", "Go wash", "gowash", "config"},
	{"fastmap", "Map the house", "fastmap", "config"},
}

func (h *haBridge) publishDiscovery() error {
	payload := map[string]any{
		"dev":  h.device(),
		"o":    map[string]any{"name": "x10-ctl"},
		"cmps": h.components(),
	}
	body, err := json.Marshal(payload)
	if err != nil {
		return err
	}
	topic := fmt.Sprintf("%s/device/%s/config", haPrefix, h.node)
	if err := h.bus.PublishRetained(topic, body); err != nil {
		return err
	}
	log.Printf("ha: published discovery for %d entities on %s", len(h.components()), topic)
	return nil
}

// publishState reads the caches the Watch* goroutines already maintain, so a
// tick costs no bus traffic.
func (h *haBridge) publishState() {
	status, _ := h.srv.local.WorkStatus()

	activity, ok := haActivity[status]
	if !ok {
		activity = "idle"
	}
	state := map[string]any{"state": activity}

	set, err := readSettings()
	if err == nil {
		if set.Fan != nil && *set.Fan >= 0 && *set.Fan < len(suctionNames) {
			state["fan_speed"] = suctionNames[*set.Fan]
		}
		if set.MopWater != nil && *set.MopWater >= 0 && *set.MopWater < len(mopNames) {
			h.pub(h.base+"/water_level", mopNames[*set.MopWater])
		}
		if set.CleanType != nil && *set.CleanType >= 0 && *set.CleanType < len(typeNames) {
			h.pub(h.base+"/clean_type", typeNames[*set.CleanType])
		}
	}

	if body, err := json.Marshal(state); err == nil {
		h.pub(h.base+"/state", string(body))
	}

	if bat := h.srv.local.Battery(); bat.Valid {
		h.pub(h.base+"/battery", fmt.Sprint(bat.Level))
		// 0 off dock, 1 charging, 2 on dock and full. 1 is inferred, not
		// observed, so this can read OFF on a robot that is in fact charging.
		charging := "OFF"
		if bat.ChargeState == chargeStateCharging {
			charging = "ON"
		}
		h.pub(h.base+"/charging", charging)
	}
	if status != "" {
		h.pub(h.base+"/status", status)
	}
}

// onCommand routes one HA command. Topics are ours, so the suffix is enough.
func (h *haBridge) onCommand(topic string, payload []byte) {
	body := strings.TrimSpace(string(payload))
	// HA restarted. Its entity registry may be empty and only we can refill it.
	if topic == haPrefix+"/status" {
		if body == haOnline {
			log.Print("ha: Home Assistant came up, republishing discovery")
			h.onConnect()
		}
		return
	}
	suffix := strings.TrimPrefix(topic, h.base+"/cmd/")
	ctx, cancel := context.WithTimeout(context.Background(), 15*time.Second)
	defer cancel()

	// Every inbound command, logged before it is acted on. Without this there
	// is no way to tell a command that never arrived from one the robot ignored.
	log.Printf("ha: <- %s %q", suffix, body)

	var err error
	switch {
	case suffix == "vacuum":
		err = h.vacuumCommand(ctx, body)
	case suffix == "fan_speed":
		err = h.setIndex(ctx, "fan", suctionNames, body)
	case suffix == "water_level":
		err = h.setIndex(ctx, "mop_water", mopNames, body)
	case suffix == "clean_type":
		err = h.setIndex(ctx, "clean_type", typeNames, body)
	case strings.HasPrefix(suffix, "button/"):
		err = h.pressButton(ctx, strings.TrimPrefix(suffix, "button/"))
	case suffix == "clean_segments":
		err = h.cleanSegments(ctx, body)
	case suffix == "send":
		err = h.sendCommand(ctx, body)
	default:
		return
	}
	if err != nil {
		log.Printf("ha: %s %q FAILED: %v", suffix, body, err)
		return
	}
	log.Printf("ha: %s %q ok", suffix, body)
	// The robot reports on its own schedule; nudge so HA is not left stale.
	go func() {
		time.Sleep(2 * time.Second)
		h.publishState()
	}()
}

func (h *haBridge) vacuumCommand(ctx context.Context, cmd string) error {
	switch strings.ToLower(cmd) {
	case "start", "clean", "turn_on":
		if h.wasPaused() {
			return h.method(ctx, "resume")
		}
		return h.startClean(ctx)
	case "stop", "turn_off":
		// The app's stop is GOHOME, captured 2026-08-07. Method 12 is exit
		// remote control, which is why it never stopped a clean.
		h.setPaused(false)
		return h.method(ctx, "gohome")
	case "pause":
		if err := h.method(ctx, "pause"); err != nil {
			return err
		}
		h.setPaused(true)
		return nil
	case "return_to_base":
		return h.method(ctx, "gohome")
	case "locate":
		return h.locate(ctx)
	}
	return fmt.Errorf("unknown vacuum command")
}

func (h *haBridge) method(ctx context.Context, name string) error {
	m, ok := client.Methods[name]
	if !ok {
		return fmt.Errorf("no method %q", name)
	}
	_, err := h.srv.runMethod(ctx, name, m)
	return err
}

func (h *haBridge) pressButton(ctx context.Context, id string) error {
	for _, b := range haButtons {
		if b.id != id {
			continue
		}
		if fn, ok := dpCommands[b.cmd]; ok {
			_, err := fn(h.srv, ctx)
			return err
		}
		return h.method(ctx, b.cmd)
	}
	return fmt.Errorf("no button %q", id)
}

// setIndex turns an HA option string back into its enum value.
func (h *haBridge) setIndex(ctx context.Context, field string, names []string, value string) error {
	idx := -1
	for i, n := range names {
		if n == value {
			idx = i
			break
		}
	}
	if idx < 0 {
		return fmt.Errorf("%q is not one of %s", value, strings.Join(names, ", "))
	}
	var s CleanSettings
	switch field {
	case "fan":
		s.Fan = &idx
	case "mop_water":
		s.MopWater = &idx
	case "clean_type":
		s.CleanType = &idx
	default:
		return fmt.Errorf("no field %q", field)
	}
	return h.srv.local.SetCleanParams(ctx, &s)
}

// sendCommand is the escape hatch for everything with no entity. The payload is
// either a bare command name or {"command":"...","rooms":[1,2]}.
func (h *haBridge) sendCommand(ctx context.Context, body string) error {
	var req struct {
		Command string `json:"command"`
		Rooms   []int  `json:"rooms"`
	}
	if strings.HasPrefix(body, "{") {
		if err := json.Unmarshal([]byte(body), &req); err != nil {
			return err
		}
	} else {
		req.Command = body
	}
	if len(req.Rooms) > 0 {
		return h.cleanRooms(ctx, req.Rooms)
	}
	if fn, ok := dpCommands[strings.ToLower(req.Command)]; ok {
		_, err := fn(h.srv, ctx)
		return err
	}
	return h.method(ctx, strings.ToLower(req.Command))
}

// startClean cleans every room on the map.
//
// Not method 0. AUTO_CLEAN was fired from Home Assistant on 2026-08-07 and the
// robot did not move - client.go had already recorded that only GOHOME was
// confirmed on hardware. Room clean carries a param encoder that is proven, so
// "clean the house" is every room rather than an enum value that does nothing.
func (h *haBridge) startClean(ctx context.Context) error {
	rooms, _, err := readRooms()
	if err != nil {
		return fmt.Errorf("no room list, so no whole-home clean: %w", err)
	}
	if len(rooms) == 0 {
		return fmt.Errorf("the map has no rooms")
	}
	ids := make([]int, 0, len(rooms))
	for _, r := range rooms {
		ids = append(ids, r.ID)
	}
	h.setPaused(false)
	log.Printf("ha: start -> room clean of %v", ids)
	return h.cleanRooms(ctx, ids)
}

// Rooms are sorted so a repeated request produces identical bytes.
func (h *haBridge) cleanRooms(ctx context.Context, rooms []int) error {
	sort.Ints(rooms)
	meta, err := readMapMeta()
	if err != nil {
		return err
	}
	param := roomsParam(rooms, uint64(meta.ID), 1)
	_, err = h.srv.modeCtrlParamCtx(ctx, dp.MethodRoomClean, param)
	return err
}

// publishStation reads the dock. Each read spawns akostopic, so it runs on its
// own slow tick rather than with the rest of the state.
func (h *haBridge) publishStation() {
	// The read is intermittent: akostopic occasionally returns nothing inside
	// its window, and a miss would leave the entities blank for a whole tick.
	var st StationState
	var err error
	for attempt := 0; attempt < 3; attempt++ {
		if attempt > 0 {
			time.Sleep(2 * time.Second)
		}
		ctx, cancel := context.WithTimeout(context.Background(), 20*time.Second)
		st, err = h.srv.local.StationStatus(ctx)
		cancel()
		if err == nil && st.Valid {
			break
		}
	}
	if err != nil || !st.Valid {
		log.Printf("ha: station: no reading after 3 tries (%v)", err)
		return
	}
	h.pub(h.base+"/water_tank", fmt.Sprint(st.WaterPercent))
	h.pub(h.base+"/water_empty", haOnOff(st.ErrorCode&stationErrWaterEmpty != 0))
	h.pub(h.base+"/wastewater", haOnOff(st.ErrorCode&stationErrWasteFault != 0))
	h.pub(h.base+"/dust_bag", haOnOff(st.ErrorCode&stationErrNoDustBag != 0))
	h.pub(h.base+"/dock_busy", haOnOff(st.Work != 0))
}

func haOnOff(v bool) string {
	if v {
		return "ON"
	}
	return "OFF"
}

// Run publishes state until stop closes.
func (h *haBridge) Run(every time.Duration, stop <-chan struct{}) {
	t := time.NewTicker(every)
	defer t.Stop()
	slow := time.NewTicker(haStationEvery)
	defer slow.Stop()
	for {
		select {
		case <-stop:
			// A clean shutdown beats waiting for the will to fire.
			_ = h.bus.PublishRetained(h.availTopic(), []byte(haOffline))
			return
		case <-t.C:
			if h.bus != nil {
				h.publishState()
			}
		case <-slow.C:
			if h.bus != nil {
				h.publishStation()
			}
		}
	}
}

type haConfig struct {
	srv      *server
	sn       string
	disc     *client.Discoverer
	broker   string
	ca       string
	insecure bool
	plain    bool
	user     string
	pass     string
	every    time.Duration
}

// pick returns the first non-empty string, so --ha-broker falls back to
// --mqtt-broker rather than making the user repeat it.
func pick(a, b string) string {
	if a != "" {
		return a
	}
	return b
}

// startHA brings the bridge up on its own broker connection.
func startHA(cfg *haConfig, stop <-chan struct{}) {
	if cfg.broker == "" {
		log.Print("ha: --ha given with no broker; set --ha-broker or --mqtt-broker")
		return
	}
	go func() {
		sn := waitForSerial(cfg, stop)
		if sn == "" {
			return
		}
		bridgeUp(cfg, sn, stop)
	}()
}

// waitForSerial blocks until discovery learns the serial. The init script
// passes no --sn, and at boot no UDP 9667 broadcast has been heard yet.
func waitForSerial(cfg *haConfig, stop <-chan struct{}) string {
	if cfg.sn != "" {
		return cfg.sn
	}
	t := time.NewTicker(5 * time.Second)
	defer t.Stop()
	for warned := false; ; {
		if cfg.disc != nil {
			if last := cfg.disc.Last(); last != nil && last.DeviceSN != "" {
				return last.DeviceSN
			}
		}
		if !warned {
			log.Print("ha: waiting for discovery to learn the serial")
			warned = true
		}
		select {
		case <-stop:
			return ""
		case <-t.C:
		}
	}
}

func bridgeUp(cfg *haConfig, sn string, stop <-chan struct{}) {
	h := newHABridge(cfg.srv, sn)
	bus := mqtt.NewClientBus(cfg.broker, "x10-ctl-ha-"+sn, cfg.ca, cfg.insecure, cfg.plain, nil)
	bus.Opts.Username, bus.Opts.Password = cfg.user, cfg.pass
	h.attach(bus)

	// With --mqtt off there is no data-point path, and some actions only exist
	// there. cloud_cli is on this same broker, so lend it this connection.
	// HasBus, not Ready: an internal broker is a real path before any client
	// dials in, and lending here would replace it and route DPs off-device.
	if cfg.srv.cloud != nil && !cfg.srv.cloud.HasBus() {
		cfg.srv.cloud.SetIdentity(sn)
		cfg.srv.cloud.SetBus(bus)
		log.Print("ha: data-point path available over the bridge connection")
	}

	go bus.Run(stop)
	go h.Run(cfg.every, stop)
	log.Printf("ha: bridging to %s as %s, state every %s", cfg.broker, h.node, cfg.every)
}

// locate beeps and then stops. Beep is a state on this robot - audio_cli -o
// starts it and only -s ends it - while HA's locate is one-shot.
func (h *haBridge) locate(ctx context.Context) error {
	if _, err := h.srv.beep(ctx, true); err != nil {
		return err
	}
	go func() {
		time.Sleep(haLocateBeep)
		off, cancel := context.WithTimeout(context.Background(), 10*time.Second)
		defer cancel()
		if _, err := h.srv.beep(off, false); err != nil {
			log.Printf("ha: locate: could not stop the beep: %v", err)
		}
	}()
	return nil
}

// work_status has no paused, so a pause is remembered here. Without it HA's
// start - which has no separate resume payload - restarts the run from scratch.
func (h *haBridge) setPaused(v bool) {
	h.mu.Lock()
	h.paused = v
	h.mu.Unlock()
}

func (h *haBridge) wasPaused() bool {
	h.mu.Lock()
	defer h.mu.Unlock()
	return h.paused
}

// cleanSegments is HA's native room selection: a JSON array of room ids.
func (h *haBridge) cleanSegments(ctx context.Context, body string) error {
	var raw []any
	if err := json.Unmarshal([]byte(body), &raw); err != nil {
		return fmt.Errorf("clean_segments wants a JSON array: %w", err)
	}
	rooms := make([]int, 0, len(raw))
	for _, v := range raw {
		switch n := v.(type) {
		case float64:
			rooms = append(rooms, int(n))
		case string:
			id, err := strconv.Atoi(n)
			if err != nil {
				return fmt.Errorf("segment %q is not a room id", n)
			}
			rooms = append(rooms, id)
		default:
			return fmt.Errorf("segment %v is neither a number nor a string", v)
		}
	}
	if len(rooms) == 0 {
		return fmt.Errorf("no segments given")
	}
	h.setPaused(false)
	return h.cleanRooms(ctx, rooms)
}
