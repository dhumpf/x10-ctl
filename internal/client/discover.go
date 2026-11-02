package client

import (
	"encoding/json"
	"fmt"
	"github.com/dhumpf/x10-ctl/internal/pb"
	"log"
	"net"
	"os"
	"path/filepath"
	"sync"
	"time"
)

// discoveryPort is where the robot announces itself on the LAN every 30 s, as a
// proto.cloud.SocketBroadcast carrying its address, serial and bind state.
// x10-ctl listens for that announcement so it can auto-configure for your robot
// rather than taking the address and serial as hand-written flags.
const DiscoveryPort = 9667

type Discovery struct {
	IP       string    `json:"ip"`
	DeviceSN string    `json:"device_sn"`
	UserID   string    `json:"user_id"`
	IsBind   bool      `json:"is_bind"`
	Seen     time.Time `json:"seen"`
}

type Discoverer struct {
	mu   sync.RWMutex
	last *Discovery
	path string // where to cache it, so a restart is not blind for up to 30 s
}

func NewDiscoverer(cachePath string) *Discoverer {
	d := &Discoverer{path: cachePath}
	d.loadCache()
	return d
}

func (d *Discoverer) Last() *Discovery {
	d.mu.RLock()
	defer d.mu.RUnlock()
	if d.last == nil {
		return nil
	}
	c := *d.last
	return &c
}

func (d *Discoverer) loadCache() {
	if d.path == "" {
		return
	}
	b, err := os.ReadFile(d.path)
	if err != nil {
		return
	}
	var c Discovery
	if json.Unmarshal(b, &c) == nil && c.DeviceSN != "" {
		d.last = &c
	}
}

func (d *Discoverer) saveCache() {
	if d.path == "" || d.last == nil {
		return
	}
	b, err := json.MarshalIndent(d.last, "", "  ")
	if err != nil {
		return
	}
	_ = os.MkdirAll(filepath.Dir(d.path), 0o755)
	// 0600: user_id is an account identifier, not something to leave world-readable.
	_ = os.WriteFile(d.path, b, 0o600)
}

// parseBroadcast decodes SocketBroadcast. Returns nil if the packet is not one.
func parseBroadcast(pkt []byte) *Discovery {
	if len(pkt) < 3 {
		return nil
	}
	// The payload is length-delimited, same as everything else on this device.
	n, i, ok := pb.ReadVarint(pkt, 0)
	if !ok || int(n) > len(pkt)-i {
		return nil
	}
	body := pkt[i : i+int(n)]
	out := &Discovery{Seen: time.Now()}
	for _, f := range pb.ParseFields(body) {
		switch {
		case f.Num == 1 && f.Wire == 0:
			out.IsBind = f.Val != 0
		case f.Num == 2 && f.Wire == 2:
			out.DeviceSN = string(f.Data)
		case f.Num == 3 && f.Wire == 2:
			out.UserID = string(f.Data)
		}
	}
	if out.DeviceSN == "" {
		return nil
	}
	return out
}

// Run listens for broadcasts until ctx is done, pushing every fresh identity
// into the client. Errors are logged and retried rather than fatal: the robot
// may be off, and it will announce itself again within 30 s of coming back.
func (d *Discoverer) Run(c *Client, stop <-chan struct{}) {
	// Apply whatever the cache had straight away so the UI is usable on restart.
	if last := d.Last(); last != nil {
		c.SetIdentity(last.IP, last.DeviceSN, last.UserID)
		log.Printf("discovery: using cached identity %s at %s", last.DeviceSN, last.IP)
	}
	for {
		select {
		case <-stop:
			return
		default:
		}
		if err := d.listen(c, stop); err != nil {
			log.Printf("discovery: %v (retrying in 10s)", err)
			select {
			case <-stop:
				return
			case <-time.After(10 * time.Second):
			}
		}
	}
}

func (d *Discoverer) listen(c *Client, stop <-chan struct{}) error {
	pc, err := net.ListenPacket("udp4", fmt.Sprintf(":%d", DiscoveryPort))
	if err != nil {
		return err
	}
	defer pc.Close()
	log.Printf("discovery: listening on udp/%d", DiscoveryPort)

	buf := make([]byte, 2048)
	for {
		select {
		case <-stop:
			return nil
		default:
		}
		_ = pc.SetReadDeadline(time.Now().Add(5 * time.Second))
		n, addr, err := pc.ReadFrom(buf)
		if err != nil {
			if ne, ok := err.(net.Error); ok && ne.Timeout() {
				continue
			}
			return err
		}
		got := parseBroadcast(buf[:n])
		if got == nil {
			continue
		}
		if ua, ok := addr.(*net.UDPAddr); ok {
			got.IP = ua.IP.String()
		}
		d.mu.Lock()
		changed := d.last == nil || d.last.DeviceSN != got.DeviceSN ||
			d.last.UserID != got.UserID || d.last.IP != got.IP
		d.last = got
		d.mu.Unlock()
		if changed {
			log.Printf("discovery: %s at %s (bound=%v)", got.DeviceSN, got.IP, got.IsBind)
			d.saveCache()
		}
		c.SetIdentity(got.IP, got.DeviceSN, got.UserID)
	}
}
