//go:build linux && arm64

// mdsdump attaches an eros::mds channel segment read-only and prints it.
//
// Probe tool. Read-only by construction: shmat is always SHM_RDONLY, so a bad
// guess about the layout cannot disturb the running stack.
//
// arm64 Linux only, and the build tag is not cosmetic: the syscall numbers below
// are the asm-generic ones, and 194 means something else entirely on macOS.
//
// Segment layout, read off the live robot:
//
//	0x00 u32 shm key
//	0x48 u32 write offset within the ring
//	0x58 u32 offset of the newest record
//	0x6c u32 ring size in bytes
//	0x70      ring: repeated { u32 length, length bytes of message }
//
// Every message is { Header header = 1, ...own fields from 2 up }, with
// Header { Src src = 1, string node = 2, uint64 seq = 3, string frame_id = 4,
// string name = 5 }.
package main

import (
	"encoding/binary"
	"encoding/hex"
	"flag"
	"fmt"
	"math"
	"os"
	"syscall"
	"time"
	"unsafe"
)

const (
	sysShmget = 194
	sysShmat  = 196
	sysShmdt  = 197
	shmRDONLY = 010000
	seed      = 0xc70f6907
	m64       = 0xc6a4a7935bd1e995
	r64       = 47

	hdrLen     = 0x70
	offWrite   = 0x48
	offNewest  = 0x58
	offRing    = 0x6c
	maxPreview = 6
)

func murmur64a(key []byte, seed uint64) uint64 {
	h := seed ^ (uint64(len(key)) * m64)
	n := len(key) / 8 * 8
	for i := 0; i < n; i += 8 {
		k := binary.LittleEndian.Uint64(key[i : i+8])
		k *= m64
		k ^= k >> r64
		k *= m64
		h ^= k
		h *= m64
	}
	if rest := key[n:]; len(rest) > 0 {
		for i, b := range rest {
			h ^= uint64(b) << (8 * uint(i))
		}
		h *= m64
	}
	h ^= h >> r64
	h *= m64
	h ^= h >> r64
	return h
}

func varint(b []byte, i int) (uint64, int, bool) {
	var n uint64
	for shift := uint(0); ; shift += 7 {
		if i >= len(b) || shift > 63 {
			return 0, i, false
		}
		c := b[i]
		i++
		n |= uint64(c&0x7f) << shift
		if c&0x80 == 0 {
			return n, i, true
		}
	}
}

// walk prints one protobuf message as field number, wire type and value.
// Length-delimited fields that look like packed float32 are summarised rather
// than dumped: a LaserScan carries a few thousand of them.
func walk(b []byte, indent string) {
	i := 0
	for i < len(b) {
		tag, ni, ok := varint(b, i)
		if !ok {
			fmt.Printf("%s<trailing %d bytes>\n", indent, len(b)-i)
			return
		}
		i = ni
		f, w := tag>>3, tag&7
		switch w {
		case 0:
			v, ni, ok := varint(b, i)
			if !ok {
				return
			}
			i = ni
			fmt.Printf("%sf%-3d varint  %d\n", indent, f, v)
		case 5:
			if i+4 > len(b) {
				return
			}
			u := binary.LittleEndian.Uint32(b[i : i+4])
			fmt.Printf("%sf%-3d fixed32 %g  (0x%08x)\n", indent, f, math.Float32frombits(u), u)
			i += 4
		case 1:
			if i+8 > len(b) {
				return
			}
			u := binary.LittleEndian.Uint64(b[i : i+8])
			fmt.Printf("%sf%-3d fixed64 %g\n", indent, f, math.Float64frombits(u))
			i += 8
		case 2:
			n, ni, ok := varint(b, i)
			if !ok || ni+int(n) > len(b) {
				return
			}
			i = ni
			v := b[i : i+int(n)]
			i += int(n)
			switch {
			case n > 0 && n%4 == 0 && n >= 64:
				cnt := int(n) / 4
				fmt.Printf("%sf%-3d packed  %d float32:", indent, f, cnt)
				for k := 0; k < cnt && k < maxPreview; k++ {
					fmt.Printf(" %g", math.Float32frombits(binary.LittleEndian.Uint32(v[k*4:])))
				}
				fmt.Println(" ...")
			case printable(v):
				fmt.Printf("%sf%-3d string  %q\n", indent, f, v)
			default:
				fmt.Printf("%sf%-3d msg     %d bytes\n", indent, f, n)
				walk(v, indent+"    ")
			}
		default:
			fmt.Printf("%sf%-3d wire %d - stop\n", indent, f, w)
			return
		}
	}
}

func printable(b []byte) bool {
	if len(b) == 0 {
		return false
	}
	for _, c := range b {
		if c < 0x20 || c > 0x7e {
			return false
		}
	}
	return true
}

func main() {
	var (
		name  = flag.String("c", "", "channel name")
		rawK  = flag.Uint64("k", 0, "raw shm key instead of a channel name")
		n     = flag.Int("n", 256, "bytes to dump")
		off   = flag.Int("o", -1, "raw offset to hex dump; default is parse mode")
		last  = flag.Int("last", 1, "parse this many of the newest records")
		watch = flag.Duration("watch", 0, "repeat at this interval")
	)
	flag.Parse()

	key := *rawK
	if *name != "" {
		full := murmur64a([]byte(*name), seed)
		key = full & 0xFFFFFFFF
		fmt.Printf("channel %s\n  id  0x%016x  key 0x%08x\n", *name, full, key)
	}
	if key == 0 {
		fmt.Fprintln(os.Stderr, "need -c or -k")
		os.Exit(2)
	}

	id, _, errno := syscall.Syscall(sysShmget, uintptr(uint32(key)), 0, 0666)
	if errno != 0 {
		fmt.Fprintf(os.Stderr, "shmget 0x%08x: %v\n", key, errno)
		os.Exit(1)
	}
	addr, _, errno := syscall.Syscall(sysShmat, id, 0, shmRDONLY)
	if errno != 0 {
		fmt.Fprintf(os.Stderr, "shmat: %v\n", errno)
		os.Exit(1)
	}
	defer syscall.Syscall(sysShmdt, addr, 0, 0)
	fmt.Printf("  shmid %d\n", id)

	at := func(o, l int) []byte {
		b := make([]byte, l)
		copy(b, unsafe.Slice((*byte)(unsafe.Pointer(addr+uintptr(o))), l))
		return b
	}

	for {
		if *off >= 0 {
			fmt.Println(hex.Dump(at(*off, *n)))
		} else {
			h := at(0, hdrLen)
			ring := int(binary.LittleEndian.Uint32(h[offRing:]))
			newest := int(binary.LittleEndian.Uint32(h[offNewest:]))
			write := int(binary.LittleEndian.Uint32(h[offWrite:]))
			fmt.Printf("  ring %d  write %d  newest %d\n\n", ring, write, newest)

			// Records are laid out forwards, so the newest sits at `newest` and
			// earlier ones can only be reached by walking from the start. Show
			// the newest first, then walk forward from it for the rest.
			o := newest
			for k := 0; k < *last; k++ {
				if o < 0 || o+4 > ring {
					break
				}
				ln := int(binary.LittleEndian.Uint32(at(hdrLen+o, 4)))
				if ln <= 0 || o+4+ln > ring {
					fmt.Printf("  record @%d: implausible length %d\n", o, ln)
					break
				}
				fmt.Printf("  --- record @%d  %d bytes ---\n", o, ln)
				walk(at(hdrLen+o+4, ln), "  ")
				fmt.Println()
				o += 4 + ln
			}
		}
		if *watch == 0 {
			return
		}
		time.Sleep(*watch)
		fmt.Println("========")
	}
}
