package mqtt

// Embedded MQTT broker - the default mode.
//
// A robot configured to use a local broker opens a normal TLS session to it and
// expects a broker on the far side. Point it at this host and this listener is
// the entire cloud: no mosquitto, no second process, no network hop off the
// device.
//
// Scope is deliberately the minimal subset for a full session: CONNECT,
// SUBSCRIBE, PUBLISH, PINGREQ, DISCONNECT. The
// robot presents no client certificate, so this asks for none - see the
// tls.Config below.

import (
	"bufio"
	"crypto/ecdsa"
	"crypto/elliptic"
	"crypto/rand"
	"crypto/tls"
	"crypto/x509"
	"crypto/x509/pkix"
	"encoding/pem"
	"fmt"
	"log"
	"math/big"
	"net"
	"os"
	"sync"
	"sync/atomic"
	"time"
)

type BrokerBus struct {
	addr    string
	cert    tls.Certificate
	OnMsg   MsgHandler
	plain   bool // serve without TLS, for a local test broker
	mu      sync.RWMutex
	clients map[*brokerConn]struct{}
	count   atomic.Uint64
	since   time.Time
	lastErr string
	bound   string // the address actually listening, once serve() is up
	nextID  atomic.Uint32
}

// Addr is the listening address, resolved. Only interesting when the requested
// port was 0.
func (b *BrokerBus) Addr() string {
	b.mu.RLock()
	defer b.mu.RUnlock()
	if b.bound != "" {
		return b.bound
	}
	return b.addr
}

type brokerConn struct {
	conn     net.Conn
	w        sync.Mutex // one writer at a time; publishes race with acks
	clientID string
	subs     []string
}

func NewBrokerBus(addr, certFile, keyFile string, plain bool, OnMsg MsgHandler) (*BrokerBus, error) {
	b := &BrokerBus{addr: addr, OnMsg: OnMsg, plain: plain, clients: map[*brokerConn]struct{}{}}
	if plain {
		return b, nil
	}
	cert, err := loadOrCreateCert(certFile, keyFile)
	if err != nil {
		return nil, err
	}
	b.cert = cert
	return b, nil
}

func (b *BrokerBus) Status() BusStatus {
	b.mu.RLock()
	defer b.mu.RUnlock()
	endpoint := b.bound
	if endpoint == "" {
		endpoint = b.addr
	}
	st := BusStatus{
		Mode: "internal", Endpoint: endpoint, Connected: len(b.clients) > 0,
		Messages: b.count.Load(), Since: stamp(b.since), LastError: b.lastErr,
	}
	for c := range b.clients {
		st.ClientID = c.clientID
		break
	}
	return st
}

func (b *BrokerBus) setErr(err error) {
	b.mu.Lock()
	defer b.mu.Unlock()
	if err == nil {
		b.lastErr = ""
		return
	}
	b.lastErr = err.Error()
}

// Publish pushes to every connected client whose subscriptions match. In
// practice that is the one robot, but honouring the filter keeps the broker
// correct if another client attaches at the same time.
func (b *BrokerBus) Publish(topic string, payload []byte) error {
	// Never retained: this broker keeps no retained store.
	pkt := makePublish(topic, payload, uint16(b.nextID.Add(1)), 0, false).encode()
	b.mu.RLock()
	targets := make([]*brokerConn, 0, len(b.clients))
	for c := range b.clients {
		if c.matches(topic) {
			targets = append(targets, c)
		}
	}
	total := len(b.clients)
	b.mu.RUnlock()

	if len(targets) == 0 {
		if total == 0 {
			return fmt.Errorf("no MQTT client connected to the internal broker on %s "+
				"(is the robot configured to use this broker?)", b.addr)
		}
		return fmt.Errorf("connected client is not subscribed to %s", topic)
	}
	var firstErr error
	for _, c := range targets {
		if err := c.write(pkt); err != nil && firstErr == nil {
			firstErr = err
		}
	}
	return firstErr
}

func (b *BrokerBus) Run(stop <-chan struct{}) {
	for {
		select {
		case <-stop:
			return
		default:
		}
		if err := b.serve(stop); err != nil {
			b.setErr(err)
			log.Printf("mqtt broker: %v (retrying in 10s)", err)
			select {
			case <-stop:
				return
			case <-time.After(10 * time.Second):
			}
		}
	}
}

func (b *BrokerBus) serve(stop <-chan struct{}) error {
	var (
		ln  net.Listener
		err error
	)
	if b.plain {
		ln, err = net.Listen("tcp", b.addr)
	} else {
		// No client certificate is requested. The robot never offers one, and
		// demanding it would break the handshake before a byte of MQTT.
		ln, err = tls.Listen("tcp", b.addr, &tls.Config{
			Certificates: []tls.Certificate{b.cert},
			MinVersion:   tls.VersionTLS12,
		})
	}
	if err != nil {
		return err
	}
	defer ln.Close()

	b.mu.Lock()
	b.since = time.Now()
	b.lastErr = ""
	b.bound = ln.Addr().String() // resolved, so a :0 port is discoverable
	b.mu.Unlock()
	log.Printf("mqtt broker: listening on %s (tls=%v)", ln.Addr(), !b.plain)

	go func() {
		<-stop
		_ = ln.Close()
	}()

	for {
		conn, err := ln.Accept()
		if err != nil {
			select {
			case <-stop:
				return nil
			default:
			}
			return err
		}
		go b.handle(conn)
	}
}

func (b *BrokerBus) handle(conn net.Conn) {
	defer conn.Close()
	c := &brokerConn{conn: conn}
	log.Printf("mqtt broker: connect from %s", conn.RemoteAddr())

	b.mu.Lock()
	b.clients[c] = struct{}{}
	b.mu.Unlock()
	defer func() {
		b.mu.Lock()
		delete(b.clients, c)
		b.mu.Unlock()
		log.Printf("mqtt broker: %s disconnected", conn.RemoteAddr())
	}()

	r := bufio.NewReader(conn)
	for {
		// Generous: the robot's keepalive is its own business, and a docked
		// robot can go quiet for a long time between reports.
		_ = conn.SetReadDeadline(time.Now().Add(10 * time.Minute))
		pkt, err := readPacket(r)
		if err != nil {
			return
		}
		if err := b.dispatch(c, pkt); err != nil {
			log.Printf("mqtt broker: %v", err)
			return
		}
	}
}

func (b *BrokerBus) dispatch(c *brokerConn, pkt packet) error {
	switch pkt.kind {
	case pktCONNECT:
		id, _, err := parseConnect(pkt)
		if err != nil {
			return err
		}
		c.clientID = id
		log.Printf("mqtt broker: CONNACK to client_id=%q", id)
		return c.write(makeConnack(true).encode())

	case pktSUBSCRIBE:
		id, filters, err := parseSubscribe(pkt)
		if err != nil {
			return err
		}
		c.subs = append(c.subs, filters...)
		log.Printf("mqtt broker: SUBACK %v", filters)
		return c.write(makeSuback(id, len(filters)).encode())

	case pktUNSUBSCRIBE:
		id, _, ok := readUint16(pkt.body, 0)
		if !ok {
			return errShortPacket
		}
		return c.write(packet{kind: pktUNSUBACK, body: []byte{byte(id >> 8), byte(id)}}.encode())

	case pktPUBLISH:
		msg, err := parsePublish(pkt)
		if err != nil {
			return err
		}
		if msg.QoS > 0 {
			if err := c.write(makePuback(msg.ID).encode()); err != nil {
				return err
			}
		}
		b.count.Add(1)
		if b.OnMsg != nil {
			b.OnMsg(msg.Topic, msg.Payload)
		}
		return nil

	case pktPINGREQ:
		return c.write(packet{kind: pktPINGRESP}.encode())

	case pktDISCONNECT:
		return fmt.Errorf("client %q sent DISCONNECT", c.clientID)
	}
	return nil // unknown types are ignored, not fatal
}

func (c *brokerConn) matches(topic string) bool {
	for _, f := range c.subs {
		if topicMatches(f, topic) {
			return true
		}
	}
	return false
}

func (c *brokerConn) write(b []byte) error {
	c.w.Lock()
	defer c.w.Unlock()
	_ = c.conn.SetWriteDeadline(time.Now().Add(10 * time.Second))
	_, err := c.conn.Write(b)
	return err
}

// loadOrCreateCert reads the broker certificate, generating a self-signed leaf
// on first run so the broker always starts. The robot checks the chain
// so it accepts this broker only if it is configured to trust this certificate;
// how you arrange that on your own device is out of scope here.
func loadOrCreateCert(certFile, keyFile string) (tls.Certificate, error) {
	if certFile == "" || keyFile == "" {
		return tls.Certificate{}, fmt.Errorf("internal broker needs --mqtt-cert and --mqtt-key, or --mqtt-plain")
	}
	if _, err := os.Stat(certFile); err == nil {
		return tls.LoadX509KeyPair(certFile, keyFile)
	}

	key, err := ecdsa.GenerateKey(elliptic.P256(), rand.Reader)
	if err != nil {
		return tls.Certificate{}, err
	}
	serial, err := rand.Int(rand.Reader, new(big.Int).Lsh(big.NewInt(1), 128))
	if err != nil {
		return tls.Certificate{}, err
	}
	tpl := &x509.Certificate{
		SerialNumber:          serial,
		Subject:               pkix.Name{CommonName: "x10-ctl"},
		NotBefore:             time.Now().Add(-time.Hour),
		NotAfter:              time.Now().AddDate(10, 0, 0),
		KeyUsage:              x509.KeyUsageDigitalSignature,
		ExtKeyUsage:           []x509.ExtKeyUsage{x509.ExtKeyUsageServerAuth},
		BasicConstraintsValid: true,
		IsCA:                  false,
		DNSNames:              []string{"localhost", "x10-ctl"},
		IPAddresses:           []net.IP{net.IPv4(127, 0, 0, 1), net.IPv6loopback},
	}
	der, err := x509.CreateCertificate(rand.Reader, tpl, tpl, &key.PublicKey, key)
	if err != nil {
		return tls.Certificate{}, err
	}
	keyDER, err := x509.MarshalECPrivateKey(key)
	if err != nil {
		return tls.Certificate{}, err
	}
	certPEM := pem.EncodeToMemory(&pem.Block{Type: "CERTIFICATE", Bytes: der})
	keyPEM := pem.EncodeToMemory(&pem.Block{Type: "EC PRIVATE KEY", Bytes: keyDER})
	if err := os.WriteFile(certFile, certPEM, 0o644); err != nil {
		return tls.Certificate{}, err
	}
	if err := os.WriteFile(keyFile, keyPEM, 0o600); err != nil {
		return tls.Certificate{}, err
	}
	log.Printf("mqtt broker: generated self-signed %s; the robot must be "+
		"configured to trust it", certFile)
	return tls.X509KeyPair(certPEM, keyPEM)
}
