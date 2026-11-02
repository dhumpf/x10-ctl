//go:build linux && arm64

package app

// eros::mds, read natively.
//
// mds is the production IPC layer: one preallocated SysV segment per channel,
// a ring of protobuf records inside it, and no broker. akos - the topic bus
// akostopic reads - is the debug/rviz layer sitting alongside it.
//
// Reading a channel used to mean spawning `akostopic echo`, which costs 36 MB
// RSS because it attaches the 16 MB discovery segment, and only works for
// channels some node happens to advertise. Attaching the segment directly costs
// a shmat and works for every channel in the registry.
//
// Segment layout, read off the running robot rather than from any header:
//
//	0x00 u32  shm key, echoing the low 32 bits of the channel id
//	0x48 u32  write cursor within the ring
//	0x58 u32  offset of the newest record
//	0x6c u32  ring size in bytes
//	0x70      ring: repeated { u32 length; length bytes of message }
//
// Verified by arithmetic on three channels: for BatteryInfo the newest-record
// offset plus that record's framed length equals the write cursor exactly
// (1993 + 40 = 2033), and the ring size plus the 112-byte header equals the
// segment size ipcs reports (16384 + 112 = 16496).
//
// Every message is
//
//	{ Header header = 1; ...the channel's own fields from 2 up }
//	Header { Src src = 1; string node = 2; uint64 seq = 3;
//	         string frame_id = 4; string name = 5 }
//
// so field 1 is skipped and everything else is the payload.

import (
	"encoding/binary"
	"fmt"
	"syscall"
	"unsafe"
)

const (
	mdsHdrLen    = 0x70
	mdsOffNewest = 0x58
	mdsOffRing   = 0x6c
)

// mdsNewest copies the newest record off a channel.
//
// The ring has no lock, so a record can be overwritten while it is being
// copied. The newest-record offset is the sequencing we have: read it, copy,
// read it again. Unchanged means nothing wrapped past us. LaserScan publishes
// at 7 Hz into a 400 KB ring, so a retry is rare and cheap.
func mdsNewest(channel string) ([]byte, error) {
	key := uintptr(mdsChannelKey(channel))
	id, _, errno := syscall.Syscall(sysShmget, key, 0, 0666)
	if errno != 0 {
		return nil, fmt.Errorf("mds %s (key 0x%08x): %w", channel, uint32(key), errno)
	}
	addr, _, errno := syscall.Syscall(sysShmat, id, 0, shmRDONLY)
	if errno != 0 {
		return nil, fmt.Errorf("shmat %s: %w", channel, errno)
	}
	defer syscall.Syscall(sysShmdt, addr, 0, 0)

	hdr := unsafe.Slice((*byte)(unsafe.Pointer(addr)), mdsHdrLen)
	ring := int(binary.LittleEndian.Uint32(hdr[mdsOffRing:]))
	if ring <= 0 || ring > 64<<20 {
		return nil, fmt.Errorf("mds %s: implausible ring size %d", channel, ring)
	}

	for try := 0; try < 4; try++ {
		off := int(binary.LittleEndian.Uint32(hdr[mdsOffNewest:]))
		if off < 0 || off+4 > ring {
			return nil, fmt.Errorf("mds %s: no record yet", channel)
		}
		frame := unsafe.Slice((*byte)(unsafe.Pointer(addr+uintptr(mdsHdrLen+off))), 4)
		n := int(binary.LittleEndian.Uint32(frame))
		if n <= 0 || off+4+n > ring {
			return nil, fmt.Errorf("mds %s: implausible record length %d at %d", channel, n, off)
		}
		buf := make([]byte, n)
		copy(buf, unsafe.Slice((*byte)(unsafe.Pointer(addr+uintptr(mdsHdrLen+off+4))), n))
		if int(binary.LittleEndian.Uint32(hdr[mdsOffNewest:])) == off {
			return buf, nil
		}
	}
	return nil, fmt.Errorf("mds %s: ring kept moving under the read", channel)
}

// mdsRecent returns up to max recent records, oldest first.
//
// Reading only the newest is not enough for channels whose reports are partial.
// StatusUpdateNotify is the case in point: an off-dock report carries fields 14
// and 15 with no state at all, so the newest record routinely says nothing
// about what the robot is doing and the answer is a few records back.
//
// Walks forward from the start of the ring and stops at the first implausible
// length, which is where a wrapped ring leaves a torn record. That costs the
// oldest entries after a wrap and keeps the newest, which is the right way
// round.
func mdsRecent(channel string, max int) ([][]byte, error) {
	key := uintptr(mdsChannelKey(channel))
	id, _, errno := syscall.Syscall(sysShmget, key, 0, 0666)
	if errno != 0 {
		return nil, fmt.Errorf("mds %s (key 0x%08x): %w", channel, uint32(key), errno)
	}
	addr, _, errno := syscall.Syscall(sysShmat, id, 0, shmRDONLY)
	if errno != 0 {
		return nil, fmt.Errorf("shmat %s: %w", channel, errno)
	}
	defer syscall.Syscall(sysShmdt, addr, 0, 0)

	hdr := unsafe.Slice((*byte)(unsafe.Pointer(addr)), mdsHdrLen)
	ring := int(binary.LittleEndian.Uint32(hdr[mdsOffRing:]))
	newest := int(binary.LittleEndian.Uint32(hdr[mdsOffNewest:]))
	if ring <= 0 || ring > 64<<20 {
		return nil, fmt.Errorf("mds %s: implausible ring size %d", channel, ring)
	}

	var out [][]byte
	for off := 0; off+4 <= ring; {
		n := int(binary.LittleEndian.Uint32(
			unsafe.Slice((*byte)(unsafe.Pointer(addr+uintptr(mdsHdrLen+off))), 4)))
		if n <= 0 || off+4+n > ring {
			break
		}
		buf := make([]byte, n)
		copy(buf, unsafe.Slice((*byte)(unsafe.Pointer(addr+uintptr(mdsHdrLen+off+4))), n))
		out = append(out, buf)
		if len(out) > max {
			out = out[1:]
		}
		if off == newest {
			break
		}
		off += 4 + n
	}
	if len(out) == 0 {
		return nil, fmt.Errorf("mds %s: no records", channel)
	}
	return out, nil
}

func mdsAvailable(channel string) bool {
	_, _, errno := syscall.Syscall(sysShmget, uintptr(mdsChannelKey(channel)), 0, 0666)
	return errno == 0
}

// readPathPoints reads robotapp's path buffer, keyed 0x92d203d6.
//
// Not an mds ring - no length-prefixed records, no channel name hash. A plain
// struct with the point count at 0x54 and the points from 0x58, which is why it
// needs its own reader rather than going through mdsNewest.
func readPathPoints() ([]Point, int, error) {
	id, _, errno := syscall.Syscall(sysShmget, uintptr(uint32(pathBufferKey)), 0, 0666)
	if errno != 0 {
		return nil, 0, fmt.Errorf("path buffer: %w", errno)
	}
	addr, _, errno := syscall.Syscall(sysShmat, id, 0, shmRDONLY)
	if errno != 0 {
		return nil, 0, fmt.Errorf("shmat path buffer: %w", errno)
	}
	defer syscall.Syscall(sysShmdt, addr, 0, 0)

	hdr := unsafe.Slice((*byte)(unsafe.Pointer(addr)), 0x58)
	size := int(binary.LittleEndian.Uint32(hdr[0x40:]))
	mapID := int(binary.LittleEndian.Uint32(hdr[0x4c:]))
	count := int(binary.LittleEndian.Uint32(hdr[0x54:]))
	if size <= 0 || count < 0 || 0x58+count*6 > size {
		return nil, mapID, fmt.Errorf("implausible path header: size %d count %d", size, count)
	}
	if count == 0 {
		return nil, mapID, nil
	}
	raw := make([]byte, count*6)
	copy(raw, unsafe.Slice((*byte)(unsafe.Pointer(addr+0x58)), count*6))

	pts := make([]Point, 0, count)
	for i := 0; i+6 <= len(raw); i += 6 {
		pts = append(pts, Point{
			X:    int(binary.LittleEndian.Uint16(raw[i:])),
			Y:    int(binary.LittleEndian.Uint16(raw[i+2:])),
			Flag: int(binary.LittleEndian.Uint16(raw[i+4:])),
		})
	}
	return pts, mapID, nil
}
