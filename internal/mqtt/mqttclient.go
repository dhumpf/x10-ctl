package mqtt

// External-broker mode: x10-ctl joins a broker somebody else runs (broker.example.lan)
// as an ordinary client, alongside the robot.
//
// Worth having even though the internal broker needs nothing: on a shared
// broker another client can watch the same traffic x10-ctl does while the app
// keeps working.
//
// Subscribes to `#` because after a factory reset
// the robot drops the app-name segment and publishes on cmd//T2351/<sn>/res,
// so the topic is read from live traffic rather than assumed.

import (
	"bufio"
	"crypto/tls"
	"crypto/x509"
	"fmt"
	"log"
	"net"
	"os"
	"strings"
	"sync"
	"sync/atomic"
	"time"
)

type ClientBus struct {
	addr     string
	clientID string
	caFile   string
	insecure bool
	plain    bool
	OnMsg    MsgHandler

	// Set by the Home Assistant bridge. Zero values give the cloud-session
	// behaviour: subscribe to everything, no credentials, no will.
	Topics []string
	Opts   connectOpts
	// onUp fires after SUBACK on every reconnect. The bridge republishes its
	// discovery configs there; our broker keeps no retained store.
	OnUp func()

	mu      sync.RWMutex
	conn    net.Conn
	since   time.Time
	lastErr string
	up      bool

	w      sync.Mutex
	count  atomic.Uint64
	nextID atomic.Uint32
}

func NewClientBus(addr, clientID, caFile string, insecure, plain bool, OnMsg MsgHandler) *ClientBus {
	return &ClientBus{
		addr: addr, clientID: clientID, caFile: caFile,
		insecure: insecure, plain: plain, OnMsg: OnMsg,
	}
}

func (c *ClientBus) Status() BusStatus {
	c.mu.RLock()
	defer c.mu.RUnlock()
	return BusStatus{
		Mode: "external", Endpoint: c.addr, Connected: c.up, ClientID: c.clientID,
		Since: stamp(c.since), Messages: c.count.Load(), LastError: c.lastErr,
	}
}

func (c *ClientBus) Publish(topic string, payload []byte) error {
	return c.publish(topic, payload, false)
}

// PublishRetained is the Home Assistant path: configs and state survive a
// restart of either side.
func (c *ClientBus) PublishRetained(topic string, payload []byte) error {
	return c.publish(topic, payload, true)
}

// packetID returns the next non-zero id. MQTT-2.3.1-1 forbids 0, and a plain
// uint16 truncation emits it on every 65536th publish.
func (c *ClientBus) packetID() uint16 {
	for {
		if id := uint16(c.nextID.Add(1)); id != 0 {
			return id
		}
	}
}

func (c *ClientBus) publish(topic string, payload []byte, retain bool) error {
	c.mu.RLock()
	conn, up := c.conn, c.up
	c.mu.RUnlock()
	if !up || conn == nil {
		return fmt.Errorf("not connected to the MQTT broker at %s", c.addr)
	}
	// QoS 1, matching what the robot publishes with. The PUBACK is consumed
	// by the read loop; nothing here waits for it.
	pkt := makePublish(topic, payload, c.packetID(), 1, retain).encode()
	c.w.Lock()
	defer c.w.Unlock()
	_ = conn.SetWriteDeadline(time.Now().Add(10 * time.Second))
	_, err := conn.Write(pkt)
	return err
}

func (c *ClientBus) Run(stop <-chan struct{}) {
	for {
		select {
		case <-stop:
			return
		default:
		}
		if err := c.session(stop); err != nil {
			c.mu.Lock()
			c.lastErr, c.up = err.Error(), false
			c.mu.Unlock()
			log.Printf("mqtt client: %v (retrying in 10s)", err)
		}
		select {
		case <-stop:
			return
		case <-time.After(10 * time.Second):
		}
	}
}

func (c *ClientBus) session(stop <-chan struct{}) error {
	conn, err := c.dial()
	if err != nil {
		return err
	}
	defer conn.Close()
	go func() {
		<-stop
		_ = conn.Close()
	}()

	keepalive := uint16(60)
	if _, err := conn.Write(makeConnect(c.clientID, keepalive, c.Opts).encode()); err != nil {
		return fmt.Errorf("send CONNECT: %w", err)
	}

	r := bufio.NewReader(conn)
	_ = conn.SetReadDeadline(time.Now().Add(15 * time.Second))
	ack, err := readPacket(r)
	if err != nil {
		return fmt.Errorf("read CONNACK: %w", err)
	}
	if ack.kind != pktCONNACK || len(ack.body) < 2 {
		return fmt.Errorf("expected CONNACK, got packet type %d", ack.kind)
	}
	if ack.body[1] != 0 {
		// Anker's AWS IoT policy pins the client id shape; a wrong one is
		// refused here rather than silently failing every SUBSCRIBE later.
		return fmt.Errorf("broker refused CONNECT (code %d) - client id %q rejected?", ack.body[1], c.clientID)
	}

	Topics := c.Topics
	if len(Topics) == 0 {
		Topics = []string{"#"}
	}
	if _, err := conn.Write(makeSubscribe(1, Topics...).encode()); err != nil {
		return fmt.Errorf("send SUBSCRIBE: %w", err)
	}

	c.mu.Lock()
	c.conn, c.up, c.since, c.lastErr = conn, true, time.Now(), ""
	c.mu.Unlock()
	log.Printf("mqtt client: connected to %s as %q, subscribed to %s",
		c.addr, c.clientID, strings.Join(Topics, " "))
	if c.OnUp != nil {
		// After c.up is set, so the callback can publish.
		go c.OnUp()
	}
	defer func() {
		c.mu.Lock()
		c.up, c.conn = false, nil
		c.mu.Unlock()
	}()

	pingStop := make(chan struct{})
	defer close(pingStop)
	go c.ping(conn, time.Duration(keepalive)*time.Second/2, pingStop)

	for {
		_ = conn.SetReadDeadline(time.Now().Add(time.Duration(keepalive) * 2 * time.Second))
		pkt, err := readPacket(r)
		if err != nil {
			return fmt.Errorf("connection lost: %w", err)
		}
		if pkt.kind != pktPUBLISH {
			continue // CONNACK/SUBACK/PUBACK/PINGRESP need no action
		}
		msg, err := parsePublish(pkt)
		if err != nil {
			continue
		}
		if msg.QoS > 0 {
			c.w.Lock()
			_, _ = conn.Write(makePuback(msg.ID).encode())
			c.w.Unlock()
		}
		c.count.Add(1)
		if c.OnMsg != nil {
			c.OnMsg(msg.Topic, msg.Payload)
		}
	}
}

func (c *ClientBus) ping(conn net.Conn, every time.Duration, stop <-chan struct{}) {
	t := time.NewTicker(every)
	defer t.Stop()
	for {
		select {
		case <-stop:
			return
		case <-t.C:
			c.w.Lock()
			_ = conn.SetWriteDeadline(time.Now().Add(10 * time.Second))
			_, err := conn.Write(packet{kind: pktPINGREQ}.encode())
			c.w.Unlock()
			if err != nil {
				return
			}
		}
	}
}

func (c *ClientBus) dial() (net.Conn, error) {
	d := &net.Dialer{Timeout: 10 * time.Second}
	if c.plain {
		return d.Dial("tcp", c.addr)
	}
	cfg := &tls.Config{InsecureSkipVerify: c.insecure} //nolint:gosec // opt-in via --mqtt-insecure
	if c.caFile != "" {
		pem, err := os.ReadFile(c.caFile)
		if err != nil {
			return nil, fmt.Errorf("read CA %s: %w", c.caFile, err)
		}
		pool := x509.NewCertPool()
		if !pool.AppendCertsFromPEM(pem) {
			return nil, fmt.Errorf("no certificates found in %s", c.caFile)
		}
		cfg.RootCAs = pool
		// Clearing ServerName is only needed for a broker whose certificate has
		// no SAN matching the address you dial; a broker with proper SANs does
		// not need it.
		if c.insecure {
			cfg.ServerName = ""
		}
	}
	return tls.DialWithDialer(d, "tcp", c.addr, cfg)
}
