package client

import (
	"encoding/hex"
	"github.com/dhumpf/x10-ctl/internal/pb"
	"testing"
)

// The only 9668 frame ever confirmed against hardware: GOHOME, seq=1, which
// drove the robot to its dock on 2026-07-25. If this ever stops matching, the
// encoder has drifted and nothing else in this package can be trusted.
func TestGohomeFrameMatchesHardware(t *testing.T) {
	const want = "080801120408061001"

	mcr := append(pb.FieldVarint(1, Methods["gohome"]), pb.FieldVarint(2, 1)...)
	got := hex.EncodeToString(pb.Delim(append(pb.FieldVarint(1, 1), pb.FieldBytes(2, mcr)...)))

	if got != want {
		t.Fatalf("GOHOME frame drifted\n got %s\nwant %s", got, want)
	}
}

// A real SocketBroadcast captured off the wire, with the serial and account id
// replaced by same-length placeholders.
func TestParseBroadcast(t *testing.T) {
	body := append(pb.FieldVarint(1, 1), pb.FieldBytes(2, []byte("AMP00X0E00000000"))...)
	body = append(body, pb.FieldBytes(3, []byte("0123456789abcdef0123456789abcdef01234567"))...)
	pkt := pb.Delim(body)

	d := parseBroadcast(pkt)
	if d == nil {
		t.Fatal("parseBroadcast returned nil for a valid packet")
	}
	if !d.IsBind {
		t.Error("is_bind should be true")
	}
	if d.DeviceSN != "AMP00X0E00000000" {
		t.Errorf("device_sn = %q", d.DeviceSN)
	}
	if len(d.UserID) != 40 {
		t.Errorf("user_id length = %d, want 40", len(d.UserID))
	}
}

func TestParseBroadcastRejectsGarbage(t *testing.T) {
	for _, b := range [][]byte{nil, {0x00}, {0xff, 0xff}, []byte("hello world")} {
		if got := parseBroadcast(b); got != nil {
			t.Errorf("parseBroadcast(%x) = %+v, want nil", b, got)
		}
	}
}

// The verify frame the robot accepted on 2026-07-30: nonce, serial, account id.
func TestVerifyFrameShape(t *testing.T) {
	nonce := []byte("SmZYtp7JHN9W")
	v := append(pb.FieldBytes(1, nonce), pb.FieldBytes(2, []byte("AMP00X0E00000000"))...)
	v = append(v, pb.FieldBytes(3, []byte("0123456789abcdef0123456789abcdef01234567"))...)
	frame := pb.Delim(v)

	// outer length prefix must equal the body length
	n, i, ok := pb.ReadVarint(frame, 0)
	if !ok || int(n) != len(frame)-i {
		t.Fatalf("bad length prefix: n=%d body=%d", n, len(frame)-i)
	}
	fields := pb.ParseFields(frame[i:])
	if len(fields) != 3 {
		t.Fatalf("got %d fields, want 3", len(fields))
	}
	if string(fields[0].Data) != string(nonce) {
		t.Errorf("field 1 should echo the nonce")
	}
}

func TestCommandRejectsParamMethods(t *testing.T) {
	c := NewClient("127.0.0.1", 9668, "AMP00X0E00000000", "", 0)
	if _, err := c.Command("roomsclean", 1); err == nil {
		t.Fatal("roomsclean needs a target and should be rejected, not sent bare")
	}
	if _, err := c.Command("nonsense", 1); err == nil {
		t.Fatal("unknown command should be rejected")
	}
}
