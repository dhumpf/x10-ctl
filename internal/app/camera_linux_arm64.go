//go:build linux && arm64

package app

// The camera, straight out of shared memory.
//
// camera_svc streams /dev/video11 into a SysV segment from a few seconds after
// boot and keeps doing it whether or not a cleaning job is running. The segment
// is mode 0666, so reading it needs no privilege beyond being on the device.
//
// The akos cv topics are a dead end: node_percept ships with
// publish_origin_image and publish_output_image both false, nothing subscribes
// to /topic/cv/origin/compressed, and the JpegData channel is never allocated.
// Nothing writes frames to disk either. This segment is the only live frame
// source on the robot.
//
// Layout, verified against the running camera_svc:
//
//	0x00 uint32  width      1920
//	0x04 uint32  height     1080
//	0x08 uint32  size       6220800 = 1920*1080*3
//	0x0c float64 timestamp  CLOCK_MONOTONIC seconds
//	0x1c         pixels, packed 3-byte triplets, row-major, no padding
//
// Pixels are **BGR**, not RGB. Confirmed by eye: rendered as RGB, skin reads
// blue-grey and a blue storage bin reads orange.

import (
	"encoding/binary"
	"fmt"
	"math"
	"syscall"
	"unsafe"
)

// arm64 (asm-generic) syscall numbers. Not the amd64 ones.
// 0xE0 base + MEM_FORMAT_RGB_888(2).
//
// A var, not a const: key_t is a signed 32-bit int, so 0xE2110001 does not fit
// and any constant conversion is a compile error. The kernel reads the low 32
// bits of the register, so widening an untyped uint32 gets the right key.
// ipcs prints this segment as -502202367.
var cameraShmKey = uintptr(uint32(0xE2110001))

const (
	sysShmget = 194
	sysShmat  = 196
	sysShmdt  = 197

	shmRDONLY = 010000

	cameraHdrLen = 28

	// A sane ceiling so a corrupt header cannot make us slice into nothing.
	maxFrameBytes = 32 << 20
)

type frame struct {
	W, H  int
	Pix   []byte // BGR triplets, row-major
	Stamp float64
}

// grabFrame attaches the segment read-only, copies one frame out and detaches.
//
// There is a single slot and no lock, so a copy can be torn by a writer
// mid-flight. The timestamp is the only sequencing we have: snapshot it, copy,
// and re-read it. Unchanged means the frame we hold was not overwritten while
// we were reading it. At 5 fps a retry is cheap and rarely needed.
func grabFrame() (*frame, error) {
	id, _, errno := syscall.Syscall(sysShmget, cameraShmKey, 0, 0666)
	if errno != 0 {
		return nil, fmt.Errorf("shmget camera segment: %w (is camera_svc running?)", errno)
	}
	addr, _, errno := syscall.Syscall(sysShmat, id, 0, shmRDONLY)
	if errno != 0 {
		return nil, fmt.Errorf("shmat: %w", errno)
	}
	defer syscall.Syscall(sysShmdt, addr, 0, 0)

	hdr := unsafe.Slice((*byte)(unsafe.Pointer(addr)), cameraHdrLen)
	w := int(binary.LittleEndian.Uint32(hdr[0:4]))
	h := int(binary.LittleEndian.Uint32(hdr[4:8]))
	size := int(binary.LittleEndian.Uint32(hdr[8:12]))

	if w <= 0 || h <= 0 || size <= 0 || size > maxFrameBytes || size != w*h*3 {
		return nil, fmt.Errorf("implausible camera header: %dx%d size %d", w, h, size)
	}

	full := unsafe.Slice((*byte)(unsafe.Pointer(addr)), cameraHdrLen+size)
	pix := make([]byte, size)

	var stamp float64
	for try := 0; ; try++ {
		before := binary.LittleEndian.Uint64(full[12:20])
		copy(pix, full[cameraHdrLen:])
		if binary.LittleEndian.Uint64(full[12:20]) == before {
			stamp = math.Float64frombits(before)
			break
		}
		if try == 4 {
			// Five collisions at 5 fps means something is very wrong; return
			// the frame anyway rather than failing, and let the caller see it.
			stamp = math.Float64frombits(binary.LittleEndian.Uint64(full[12:20]))
			break
		}
	}
	return &frame{W: w, H: h, Pix: pix, Stamp: stamp}, nil
}

func cameraAvailable() bool {
	id, _, errno := syscall.Syscall(sysShmget, cameraShmKey, 0, 0666)
	if errno != 0 {
		return false
	}
	_ = id
	return true
}
