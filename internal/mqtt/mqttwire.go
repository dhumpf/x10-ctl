package mqtt

// MQTT 3.1.1 on the wire - only the packet types this program needs, shared by
// the embedded broker (mqttbroker.go) and the external client (mqttclient.go).
//
// Hand-rolled rather than imported for the same reason client.go hand-rolls
// protobuf: the robot is one client speaking one dialect at QoS 0/1, and a
// general MQTT stack would be more machinery than the whole binary. Keeping
// x10-ctl dependency-free also keeps it a single static aarch64 file that drops
// into /userdata with no vendoring.
//
// Deliberately not implemented: QoS 2, retained-message storage, wills, session
// persistence. The robot uses none of them; this subset is enough for a full
// session.

import (
	"bufio"
	"encoding/binary"
	"errors"
	"fmt"
	"io"
)

// MQTT control packet types (high nibble of byte 0).
const (
	pktCONNECT     byte = 1
	pktCONNACK     byte = 2
	pktPUBLISH     byte = 3
	pktPUBACK      byte = 4
	pktSUBSCRIBE   byte = 8
	pktSUBACK      byte = 9
	pktUNSUBSCRIBE byte = 10
	pktUNSUBACK    byte = 11
	pktPINGREQ     byte = 12
	pktPINGRESP    byte = 13
	pktDISCONNECT  byte = 14
)

// maxPacket caps a single inbound packet. The largest thing the robot sends is
// a chunk of the protocol 41 bulk stream; 1 MiB is far above that and keeps a
// malformed remaining-length from turning into an allocation.
const maxPacket = 1 << 20

var errShortPacket = errors.New("mqtt: truncated packet")

type packet struct {
	kind  byte   // control packet type
	flags byte   // low nibble of byte 0: dup, qos, retain
	body  []byte // variable header + payload, after the remaining-length varint
}

func (p packet) qos() byte { return (p.flags >> 1) & 3 }

// encode renders the packet, including the fixed header.
func (p packet) encode() []byte {
	out := append([]byte{p.kind<<4 | p.flags&0x0f}, encodeRemaining(len(p.body))...)
	return append(out, p.body...)
}

// encodeRemaining writes MQTT's remaining-length field. Same 7-bit scheme as a
// protobuf varint but capped at four bytes, so it does not share varint() in
// client.go.
func encodeRemaining(n int) []byte {
	var out []byte
	for {
		b := byte(n & 0x7f)
		n >>= 7
		if n != 0 {
			out = append(out, b|0x80)
			continue
		}
		return append(out, b)
	}
}

// readPacket reads exactly one packet. It blocks until a whole one arrives, so
// callers get framing for free and never see a partial message.
func readPacket(r *bufio.Reader) (packet, error) {
	first, err := r.ReadByte()
	if err != nil {
		return packet{}, err
	}
	var remaining, mult int
	for i := 0; ; i++ {
		if i == 4 {
			return packet{}, errors.New("mqtt: remaining length over four bytes")
		}
		b, err := r.ReadByte()
		if err != nil {
			return packet{}, err
		}
		remaining += int(b&0x7f) * (1 << (7 * i))
		mult++
		if b&0x80 == 0 {
			break
		}
	}
	if remaining < 0 || remaining > maxPacket {
		return packet{}, fmt.Errorf("mqtt: packet of %d bytes refused", remaining)
	}
	body := make([]byte, remaining)
	if _, err := io.ReadFull(r, body); err != nil {
		return packet{}, err
	}
	return packet{kind: first >> 4, flags: first & 0x0f, body: body}, nil
}

// --- string and integer helpers. MQTT strings are a big-endian uint16 length
// followed by UTF-8.

func mqttString(s string) []byte {
	out := make([]byte, 2, 2+len(s))
	binary.BigEndian.PutUint16(out, uint16(len(s)))
	return append(out, s...)
}

// mqttBytes is the same field for data the spec calls binary rather than a
// string. Only the will message needs it.
func mqttBytes(b []byte) []byte {
	out := make([]byte, 2, 2+len(b))
	binary.BigEndian.PutUint16(out, uint16(len(b)))
	return append(out, b...)
}

func readMQTTString(b []byte, i int) (string, int, bool) {
	if i+2 > len(b) {
		return "", i, false
	}
	n := int(binary.BigEndian.Uint16(b[i:]))
	if i+2+n > len(b) {
		return "", i, false
	}
	return string(b[i+2 : i+2+n]), i + 2 + n, true
}

func readUint16(b []byte, i int) (uint16, int, bool) {
	if i+2 > len(b) {
		return 0, i, false
	}
	return binary.BigEndian.Uint16(b[i:]), i + 2, true
}

// --- packet builders

// will is what the broker publishes for us if we drop without a DISCONNECT.
// Home Assistant availability is built on exactly this.
type Will struct {
	Topic   string
	Payload []byte
	QoS     byte
	Retain  bool
}

// connectOpts are the CONNECT fields the robot's own session never needed.
// The zero value reproduces the original bytes exactly.
type connectOpts struct {
	Username string
	Password string
	Will     *Will
}

// makeConnect builds a CONNECT with a clean session. Flag bits and payload
// order are fixed by MQTT 3.1.1 3.1.2.3 and 3.1.3; both are pinned by tests.
func makeConnect(clientID string, keepalive uint16, Opts connectOpts) packet {
	flags := byte(0x02) // clean session
	if Opts.Will != nil {
		flags |= 0x04 | (Opts.Will.QoS&3)<<3
		if Opts.Will.Retain {
			flags |= 0x20
		}
	}
	if Opts.Username != "" {
		flags |= 0x80
	}
	if Opts.Password != "" {
		flags |= 0x40
	}

	body := mqttString("MQTT")
	body = append(body, 4, flags) // protocol level 4 (3.1.1)
	body = append(body, byte(keepalive>>8), byte(keepalive))
	body = append(body, mqttString(clientID)...)
	if Opts.Will != nil {
		body = append(body, mqttString(Opts.Will.Topic)...)
		body = append(body, mqttBytes(Opts.Will.Payload)...)
	}
	if Opts.Username != "" {
		body = append(body, mqttString(Opts.Username)...)
	}
	if Opts.Password != "" {
		body = append(body, mqttString(Opts.Password)...)
	}
	return packet{kind: pktCONNECT, body: body}
}

func makeSubscribe(id uint16, Topics ...string) packet {
	body := []byte{byte(id >> 8), byte(id)}
	for _, t := range Topics {
		body = append(append(body, mqttString(t)...), 0) // QoS 0
	}
	// bit 1 of the flags nibble is mandatory on SUBSCRIBE.
	return packet{kind: pktSUBSCRIBE, flags: 0x02, body: body}
}

// makePublish builds a PUBLISH. retain is explicit at every call site because
// a wrong value is invisible in testing and permanent in production.
func makePublish(topic string, payload []byte, id uint16, qos byte, retain bool) packet {
	body := mqttString(topic)
	if qos > 0 {
		body = append(body, byte(id>>8), byte(id))
	}
	flags := qos << 1
	if retain {
		flags |= 0x01
	}
	return packet{kind: pktPUBLISH, flags: flags, body: append(body, payload...)}
}

func makePuback(id uint16) packet {
	return packet{kind: pktPUBACK, body: []byte{byte(id >> 8), byte(id)}}
}

func makeConnack(accepted bool) packet {
	code := byte(0)
	if !accepted {
		code = 5 // not authorised
	}
	return packet{kind: pktCONNACK, body: []byte{0, code}}
}

func makeSuback(id uint16, count int) packet {
	body := []byte{byte(id >> 8), byte(id)}
	for i := 0; i < count; i++ {
		body = append(body, 0) // granted QoS 0
	}
	return packet{kind: pktSUBACK, body: body}
}

// --- packet parsers

type publishMsg struct {
	Topic   string
	Payload []byte
	ID      uint16
	QoS     byte
}

func parsePublish(p packet) (publishMsg, error) {
	topic, i, ok := readMQTTString(p.body, 0)
	if !ok {
		return publishMsg{}, errShortPacket
	}
	msg := publishMsg{Topic: topic, QoS: p.qos()}
	if msg.QoS > 0 {
		var id uint16
		if id, i, ok = readUint16(p.body, i); !ok {
			return publishMsg{}, errShortPacket
		}
		msg.ID = id
	}
	msg.Payload = p.body[i:]
	return msg, nil
}

// parseConnect pulls the client id out of a CONNECT. Everything before it is
// fixed-width apart from the protocol name, so nothing else needs decoding.
func parseConnect(p packet) (clientID string, keepalive uint16, err error) {
	_, i, ok := readMQTTString(p.body, 0) // protocol name
	if !ok {
		return "", 0, errShortPacket
	}
	i += 2 // protocol level, connect flags
	if keepalive, i, ok = readUint16(p.body, i); !ok {
		return "", 0, errShortPacket
	}
	if clientID, _, ok = readMQTTString(p.body, i); !ok {
		return "", 0, errShortPacket
	}
	return clientID, keepalive, nil
}

// parseSubscribe returns the requested topic filters and the packet id.
func parseSubscribe(p packet) (id uint16, filters []string, err error) {
	id, i, ok := readUint16(p.body, 0)
	if !ok {
		return 0, nil, errShortPacket
	}
	for i < len(p.body) {
		var f string
		if f, i, ok = readMQTTString(p.body, i); !ok {
			return 0, nil, errShortPacket
		}
		if i >= len(p.body) {
			return 0, nil, errShortPacket
		}
		i++ // requested QoS
		filters = append(filters, f)
	}
	return id, filters, nil
}

// topicMatches applies MQTT filter rules: `+` covers one level, `#` covers the
// rest. The robot subscribes to fixed topics and we subscribe to `#`, but the
// broker has to be correct for both or our own publishes go nowhere.
func topicMatches(filter, topic string) bool {
	f, t := splitTopic(filter), splitTopic(topic)
	for i := range f {
		if f[i] == "#" {
			return true
		}
		if i >= len(t) {
			return false
		}
		if f[i] != "+" && f[i] != t[i] {
			return false
		}
	}
	return len(f) == len(t)
}

func splitTopic(s string) []string {
	out := []string{}
	start := 0
	for i := 0; i < len(s); i++ {
		if s[i] == '/' {
			out = append(out, s[start:i])
			start = i + 1
		}
	}
	return append(out, s[start:])
}
