// Package main - local control client for the eufy X10 Pro Omni over the 9668
// TLS socket.
//
// No cloud, no client cert, no key. Auth is a handshake on the socket: the robot
// sends a 12-byte nonce and the client echoes it back with the device serial and
// account id it is configured for (see discover.go). Commands are a
// length-delimited SocketTransData{type=E_DP, field2=ModeCtrlRequest}.
package client

import (
	"crypto/tls"
	"encoding/hex"
	"fmt"
	"github.com/dhumpf/x10-ctl/internal/pb"
	"net"
	"strings"
	"sync"
	"time"
)

// Method values from ModeCtrlRequest.Method in control.proto. Only GOHOME is
// physically confirmed on hardware; the rest are read out of the enum and are
// untested per-unit. 11 and 19 are gaps in the enum upstream, not omissions.
var Methods = map[string]uint64{
	"autoclean": 0, "rcclean": 5, "gohome": 6, "fastmap": 9, "gowash": 10,
	"stop": 12, "pause": 13, "resume": 14, "stopgohome": 15, "stoprcclean": 16,
	"stopgowash": 17, "stopsmartfollow": 18, "cruise": 20, "mapthenclean": 25,
	// 26 was not in the recovered enum at all. Captured from the app on
	// 2026-08-07 as carpet clean; it needs a param, see MethodsNeedParam.
	"carpetclean": 26,
}

// Same enum, but these carry a required oneof Param (rooms, zones, a point), so
// sending them bare gives the robot no target. Rejected up front rather than
// firing a command that cannot mean anything.
var MethodsNeedParam = map[string]uint64{
	"roomsclean": 1, "zonesclean": 2, "spotclean": 3, "gotoclean": 4,
	"pointcruise": 21, "zonescruise": 22, "sceneclean": 24, "carpetclean": 26,
}

// --- client

type Client struct {
	mu       sync.RWMutex
	host     string
	port     int
	deviceSN string
	userID   string
	timeout  time.Duration
}

func NewClient(host string, port int, deviceSN, userID string, timeout time.Duration) *Client {
	return &Client{host: host, port: port, deviceSN: deviceSN, userID: userID, timeout: timeout}
}

// SetIdentity updates the robot identity, e.g. from a discovery broadcast.
func (c *Client) SetIdentity(host, deviceSN, userID string) {
	c.mu.Lock()
	defer c.mu.Unlock()
	if host != "" {
		c.host = host
	}
	if deviceSN != "" {
		c.deviceSN = deviceSN
	}
	c.userID = userID
}

func (c *Client) Identity() (host string, port int, sn, uid string) {
	c.mu.RLock()
	defer c.mu.RUnlock()
	return c.host, c.port, c.deviceSN, c.userID
}

func (c *Client) Configured() bool {
	_, _, sn, _ := c.Identity()
	return sn != ""
}

// session dials, completes TLS and SocketVerify, and hands back the live
// connection plus the verify reply.
//
// The certificate is a self-signed www.example.com placeholder and the robot
// requests no client certificate, so verification is skipped deliberately -
// there is no identity here to check against. Confidentiality on a LAN hop is
// all this TLS ever provided.
func (c *Client) session() (*tls.Conn, []byte, error) {
	host, port, sn, uid := c.Identity()
	if sn == "" {
		return nil, nil, fmt.Errorf("no device_sn: waiting for a discovery broadcast on UDP %d", DiscoveryPort)
	}
	addr := net.JoinHostPort(host, fmt.Sprint(port))
	d := &net.Dialer{Timeout: c.timeout}
	conn, err := tls.DialWithDialer(d, "tcp", addr, &tls.Config{InsecureSkipVerify: true})
	if err != nil {
		return nil, nil, fmt.Errorf("connect %s: %w", addr, err)
	}
	_ = conn.SetDeadline(time.Now().Add(c.timeout))

	buf := make([]byte, 64)
	n, err := conn.Read(buf)
	if err != nil || n < 12 {
		conn.Close()
		return nil, nil, fmt.Errorf("short/no nonce from robot")
	}
	nonce := buf[:12]

	v := append(pb.FieldBytes(1, nonce), pb.FieldBytes(2, []byte(sn))...)
	if uid != "" {
		v = append(v, pb.FieldBytes(3, []byte(uid))...)
	}
	if _, err := conn.Write(pb.Delim(v)); err != nil {
		conn.Close()
		return nil, nil, fmt.Errorf("send verify: %w", err)
	}

	reply := make([]byte, 512)
	n, err = conn.Read(reply)
	if err != nil {
		conn.Close()
		return nil, nil, fmt.Errorf("read verify reply: %w", err)
	}
	reply = reply[:n]

	// 02 08 01 is SocketVerifyResponse{result=FAIL}. A valid reply is the ~97
	// byte ProductInfo. Fail loudly here rather than writing into a dead session.
	if n >= 3 && reply[0] == 0x02 && reply[1] == 0x08 && reply[2] == 0x01 {
		conn.Close()
		return nil, nil, fmt.Errorf("verify rejected (E_FAIL) - wrong or stale user_id? try clearing it")
	}
	return conn, reply, nil
}

type ProductInfo struct {
	Brand string `json:"brand"`
	Model string `json:"model"`
	Code  string `json:"code"`
	Name  string `json:"name"`
	// Field 8 is 24 base64 chars (16 bytes decoded), so it is not the MAC the
	// Python client labelled it as. Most likely a session or local key, so it is
	// carried but never rendered - see main.go.
	Field8 string `json:"-"`
}

// ProductInfo authenticates and returns the robot's identity. Doubles as the
// liveness and auth check: it moves nothing.
func (c *Client) ProductInfo() (*ProductInfo, error) {
	conn, reply, err := c.session()
	if err != nil {
		return nil, err
	}
	conn.Close()
	if len(reply) == 0 {
		return nil, fmt.Errorf("no verify reply")
	}
	_, i, ok := pb.ReadVarint(reply, 0) // strip the outer length prefix
	if !ok {
		return nil, fmt.Errorf("malformed reply")
	}
	pi := &ProductInfo{}
	for _, f := range pb.ParseFields(reply[i:]) {
		if f.Wire != 2 {
			continue
		}
		s := string(f.Data)
		switch f.Num {
		case 2:
			pi.Brand = s
		case 3:
			pi.Model = s
		case 4:
			pi.Code = s
		case 5:
			pi.Name = s
		case 8:
			pi.Field8 = s
		}
	}
	return pi, nil
}

type CmdResult struct {
	Command string `json:"command"`
	Sent    string `json:"sent"`
	Reply   string `json:"reply"`
	Text    string `json:"text"`
	Ack     string `json:"ack"`
	OK      bool   `json:"ok"`
}

// Command fires a ModeCtrlRequest by name.
//
// seq defaults to 1: the only 9668 frame ever physically confirmed was
// 08 08 01 12 04 08 06 10 01 (GOHOME, seq=1), and the working MQTT payload uses
// seq=1 too.
func (c *Client) Command(name string, seq uint64) (*CmdResult, error) {
	name = strings.ToLower(name)
	method, ok := Methods[name]
	if !ok {
		if _, needs := MethodsNeedParam[name]; needs {
			return nil, fmt.Errorf("%q needs a target (rooms/zones/point) and the param encoders are not written yet", name)
		}
		return nil, fmt.Errorf("unknown command %q", name)
	}
	if seq == 0 {
		seq = 1
	}
	mcr := append(pb.FieldVarint(1, method), pb.FieldVarint(2, seq)...)
	frame := pb.Delim(append(pb.FieldVarint(1, 1), pb.FieldBytes(2, mcr)...)) // SocketTransData{type=E_DP}

	conn, _, err := c.session()
	if err != nil {
		return nil, err
	}
	defer conn.Close()
	if _, err := conn.Write(frame); err != nil {
		return nil, fmt.Errorf("send: %w", err)
	}

	reply := make([]byte, 512)
	_ = conn.SetReadDeadline(time.Now().Add(c.timeout))
	n, _ := conn.Read(reply) // a timeout here is expected, see below
	reply = reply[:max(0, n)]

	text := string(reply)
	// Since the 2026-07-26 re-pair the robot acks nothing, yet the commands
	// still run - four gohome frames drove an idle robot to its dock. So a
	// missing reply is NOT a failure; only an explicit rejection is.
	ack := ""
	switch text {
	case "OK", "FAIL", "UNKNOWN":
		ack = text
	}
	return &CmdResult{
		Command: name,
		Sent:    hex.EncodeToString(frame),
		Reply:   hex.EncodeToString(reply),
		Text:    text,
		Ack:     ack,
		OK:      ack == "OK" || ack == "",
	}, nil
}

// SendRaw authenticates then sends one arbitrary SocketTransData body, for
// probing shapes the typed helpers cannot express. body must NOT carry the
// outer length prefix.
func (c *Client) SendRaw(body []byte) (*CmdResult, error) {
	frame := pb.Delim(body)
	conn, _, err := c.session()
	if err != nil {
		return nil, err
	}
	defer conn.Close()
	if _, err := conn.Write(frame); err != nil {
		return nil, fmt.Errorf("send: %w", err)
	}
	reply := make([]byte, 512)
	_ = conn.SetReadDeadline(time.Now().Add(c.timeout))
	n, _ := conn.Read(reply)
	reply = reply[:max(0, n)]
	return &CmdResult{
		Command: "raw",
		Sent:    hex.EncodeToString(frame),
		Reply:   hex.EncodeToString(reply),
		Text:    string(reply),
		OK:      true,
	}, nil
}
