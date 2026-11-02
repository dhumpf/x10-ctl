package app

// Wi-Fi: scan, join, and falling back to an access point.
//
// The point is a robot you can drop anywhere: if it cannot find a network it
// puts up its own, you join that from a phone, and this UI is already listening
// on every interface so it is simply there.
//
// The vendor ships all of it, confirmed working on the robot:
//
//	net_cli_scan                        list networks
//	net_cli_station -s <ssid> -p <pw>   join one
//	net_cli_ap      -s <ssid> -p <pw>   put up an AP   (-S stop, -c chan, -H hidden)
//
// The AP path is off unless --ap-fallback says otherwise, and that is not
// timidity. Bringing up an AP drops the station link, which is the link
// everything remote arrives over - a wrong decision here is recovered by
// walking to the robot, not by SSH. So it stays opt-in, it waits out a grace
// period first, and it gives up and rejoins if nobody turns up.

import (
	"context"
	"fmt"
	"log"
	"net"
	"net/http"
	"regexp"
	"strconv"
	"strings"
	"time"
)

const (
	netCliScan    = "/app/bin/net_cli_scan"
	netCliStation = "/app/bin/net_cli_station"
	netCliAP      = "/app/bin/net_cli_ap"
)

// AP is one scanned network.
type AP struct {
	SSID    string `json:"ssid"`
	BSSID   string `json:"bssid"`
	Channel int    `json:"channel"`
	DBm     int    `json:"dbm"`
	Secured bool   `json:"secured"`
}

// scanLine matches the numbered blocks net_cli_scan prints:
//
//  1. bssid:AA:BB:CC:00:11:33
//     ssid:ExampleWiFi
//     need_passwd:1
//     channel:11
//     dbm:-42
//
// The first line of each block carries the entry number - "1) bssid:..." - so
// the key is not always at the start of the line. Anchoring on whitespace alone
// matched every field except bssid, which meant every block was dropped for
// having no bssid to open it.
var scanKV = regexp.MustCompile(`^\s*(?:\d+\)\s*)?(bssid|ssid|need_passwd|channel|dbm)\s*:\s*(.*)$`)

func parseScan(out string) []AP {
	var aps []AP
	var cur *AP
	flush := func() {
		if cur != nil && cur.SSID != "" {
			aps = append(aps, *cur)
		}
		cur = nil
	}
	for _, line := range strings.Split(out, "\n") {
		m := scanKV.FindStringSubmatch(line)
		if m == nil {
			continue
		}
		key, val := m[1], strings.TrimSpace(m[2])
		if key == "bssid" {
			flush()
			cur = &AP{BSSID: val}
			continue
		}
		if cur == nil {
			continue
		}
		switch key {
		case "ssid":
			cur.SSID = val
		case "need_passwd":
			cur.Secured = val != "0"
		case "channel":
			cur.Channel, _ = strconv.Atoi(val)
		case "dbm":
			cur.DBm, _ = strconv.Atoi(val)
		}
	}
	flush()
	return aps
}

// ScanWiFi lists visible networks.
//
// net_cli_scan switches the radio to station mode as a side effect - it prints
// "Send set mode(station)" - so it is not free while an AP is up.
func (l *Local) ScanWiFi(ctx context.Context) ([]AP, error) {
	if !l.Available() {
		return nil, fmt.Errorf("not running on the robot")
	}
	out, err := l.run(ctx, "timeout 20 "+netCliScan)
	if err != nil && len(out) == 0 {
		return nil, fmt.Errorf("net_cli_scan: %w", err)
	}
	return parseScan(string(out)), nil
}

// JoinWiFi connects to a network.
func (l *Local) JoinWiFi(ctx context.Context, ssid, pass string) error {
	if !l.Available() {
		return fmt.Errorf("not running on the robot")
	}
	if strings.ContainsAny(ssid+pass, "'\n") {
		return fmt.Errorf("ssid and password cannot contain quotes or newlines")
	}
	argv := netCliStation + " -s '" + ssid + "'"
	if pass != "" {
		argv += " -p '" + pass + "'"
	}
	out, err := l.run(ctx, "timeout 40 "+argv)
	if err != nil {
		return fmt.Errorf("net_cli_station: %w: %s", err, strings.TrimSpace(string(out)))
	}
	return nil
}

// StartAP brings up the robot's own access point.
func (l *Local) StartAP(ctx context.Context, ssid, pass string, channel int) error {
	if !l.Available() {
		return fmt.Errorf("not running on the robot")
	}
	if strings.ContainsAny(ssid+pass, "'\n") {
		return fmt.Errorf("ssid and password cannot contain quotes or newlines")
	}
	argv := netCliAP + " -s '" + ssid + "'"
	if pass != "" {
		argv += " -p '" + pass + "'"
	}
	if channel > 0 {
		argv += " -c " + strconv.Itoa(channel)
	}
	out, err := l.run(ctx, "timeout 30 "+argv)
	if err != nil {
		return fmt.Errorf("net_cli_ap: %w: %s", err, strings.TrimSpace(string(out)))
	}
	return nil
}

func (l *Local) StopAP(ctx context.Context) error {
	out, err := l.run(ctx, "timeout 20 "+netCliAP+" -S")
	if err != nil {
		return fmt.Errorf("net_cli_ap -S: %w: %s", err, strings.TrimSpace(string(out)))
	}
	return nil
}

// wlanUp reports whether wlan0 holds a usable address.
//
// Deliberately not a ping: the whole point is a robot with no uplink, so
// reachability of anything beyond the link says nothing. An address on the
// interface is the question being asked.
func wlanUp() (string, bool) {
	ifi, err := net.InterfaceByName("wlan0")
	if err != nil {
		return "", false
	}
	addrs, err := ifi.Addrs()
	if err != nil {
		return "", false
	}
	for _, a := range addrs {
		ipnet, ok := a.(*net.IPNet)
		if !ok || ipnet.IP.IsLoopback() || ipnet.IP.To4() == nil {
			continue
		}
		if ipnet.IP.IsLinkLocalUnicast() {
			continue
		}
		return ipnet.IP.String(), true
	}
	return "", false
}

func (s *server) handleWiFi(w http.ResponseWriter, r *http.Request) {
	ctx, cancel := context.WithTimeout(r.Context(), 45*time.Second)
	defer cancel()

	if r.Method == http.MethodGet {
		ip, up := wlanUp()
		out := map[string]any{"ok": true, "wlan0": ip, "connected": up}
		if r.URL.Query().Get("scan") == "1" {
			aps, err := s.ScanWiFiCached(ctx)
			if err != nil {
				out["scan_error"] = err.Error()
			} else {
				out["networks"] = aps
			}
		}
		writeJSON(w, http.StatusOK, out)
		return
	}
	if r.Method != http.MethodPost {
		writeJSON(w, http.StatusMethodNotAllowed, map[string]string{"error": "GET or POST"})
		return
	}

	var body struct {
		SSID string `json:"ssid"`
		Pass string `json:"pass"`
	}
	if err := decodeJSON(w, r, &body); err != nil {
		writeJSON(w, http.StatusBadRequest, map[string]any{"ok": false, "error": "bad json"})
		return
	}
	if strings.TrimSpace(body.SSID) == "" {
		writeJSON(w, http.StatusBadRequest, map[string]any{"ok": false, "error": "ssid is required"})
		return
	}
	if err := s.local.JoinWiFi(ctx, body.SSID, body.Pass); err != nil {
		writeJSON(w, http.StatusOK, map[string]any{"ok": false, "error": err.Error()})
		return
	}
	ip, up := wlanUp()
	writeJSON(w, http.StatusOK, map[string]any{"ok": true, "ssid": body.SSID, "wlan0": ip, "connected": up})
}

// ScanWiFiCached rate-limits scans. A scan takes the radio for several seconds
// and flips it to station mode, so a page that polls must not trigger one per
// request.
func (s *server) ScanWiFiCached(ctx context.Context) ([]AP, error) {
	s.scanMu.Lock()
	defer s.scanMu.Unlock()
	if time.Since(s.scanAt) < 20*time.Second && s.scanned != nil {
		return s.scanned, nil
	}
	aps, err := s.local.ScanWiFi(ctx)
	if err != nil {
		return nil, err
	}
	s.scanned, s.scanAt = aps, time.Now()
	return aps, nil
}

// WatchWiFi puts up an access point when the station link does not come back.
//
// Off unless --ap-fallback is set. The grace period exists because wlan0 is
// routinely down for a few seconds around boot and roaming, and reacting to
// that by tearing down the link would be worse than the problem.
func (s *server) WatchWiFi(stop <-chan struct{}, ssid, pass string, grace time.Duration) {
	if !s.local.Available() {
		return
	}
	log.Printf("wifi: ap fallback armed, ssid %q after %s without a link", ssid, grace)
	var downSince time.Time
	apUp := false

	tick := time.NewTicker(10 * time.Second)
	defer tick.Stop()
	for {
		select {
		case <-stop:
			if apUp {
				ctx, cancel := context.WithTimeout(context.Background(), 20*time.Second)
				_ = s.local.StopAP(ctx)
				cancel()
			}
			return
		case <-tick.C:
		}

		if _, up := wlanUp(); up {
			if apUp {
				log.Print("wifi: station link is back, dropping the ap")
				ctx, cancel := context.WithTimeout(context.Background(), 20*time.Second)
				_ = s.local.StopAP(ctx)
				cancel()
				apUp = false
			}
			downSince = time.Time{}
			continue
		}
		if downSince.IsZero() {
			downSince = time.Now()
			continue
		}
		if apUp || time.Since(downSince) < grace {
			continue
		}
		log.Printf("wifi: no link for %s, bringing up ap %q", grace, ssid)
		ctx, cancel := context.WithTimeout(context.Background(), 40*time.Second)
		if err := s.local.StartAP(ctx, ssid, pass, 6); err != nil {
			log.Printf("wifi: ap failed: %v", err)
		} else {
			apUp = true
		}
		cancel()
	}
}
