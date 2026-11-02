// Package pb holds the protobuf wire primitives the robot's messages are built
// from. The payloads are a few dozen bytes, so these hand-rolled helpers are
// less machinery than a generated runtime would be.
package pb

import (
	"encoding/binary"
	"math"
)

func Varint(n uint64) []byte {
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

func FieldVarint(num, val uint64) []byte {
	return append(Varint(num<<3|0), Varint(val)...)
}

func FieldBytes(num uint64, data []byte) []byte {
	out := append(Varint(num<<3|2), Varint(uint64(len(data)))...)
	return append(out, data...)
}

// FieldSint encodes a signed field with zigzag, as sint32/sint64 do.
func FieldSint(num uint64, val int64) []byte {
	zz := uint64(val<<1) ^ uint64(val>>63)
	return FieldVarint(num, zz)
}

// Delim wraps a message the way protobuf.js encodeDelimited does, which is what
// the robot's reader expects on this socket.
func Delim(msg []byte) []byte { return append(Varint(uint64(len(msg))), msg...) }

func ReadVarint(b []byte, i int) (uint64, int, bool) {
	var val uint64
	var shift uint
	for {
		if i >= len(b) {
			return 0, i, false
		}
		x := b[i]
		i++
		val |= uint64(x&0x7f) << shift
		if x&0x80 == 0 {
			return val, i, true
		}
		shift += 7
		if shift > 63 {
			return 0, i, false
		}
	}
}

type Field struct {
	Num  uint64
	Wire uint64
	Data []byte
	Val  uint64
}

// ParseFields walks a flat protobuf message. Truncated input yields whatever
// parsed cleanly rather than an error - the robot's replies are only ever read
// for display, and a short read should not blank the whole status card.
func ParseFields(body []byte) []Field {
	var out []Field
	i := 0
	for i < len(body) {
		tag, ni, ok := ReadVarint(body, i)
		if !ok {
			break
		}
		i = ni
		f := Field{Num: tag >> 3, Wire: tag & 7}
		switch f.Wire {
		case 0:
			v, ni, ok := ReadVarint(body, i)
			if !ok {
				return out
			}
			f.Val, i = v, ni
		case 2:
			ln, ni, ok := ReadVarint(body, i)
			if !ok || ni+int(ln) > len(body) {
				return out
			}
			f.Data, i = body[ni:ni+int(ln)], ni+int(ln)
		case 5:
			if i+4 > len(body) {
				return out
			}
			f.Data, i = body[i:i+4], i+4
		case 1:
			if i+8 > len(body) {
				return out
			}
			f.Data, i = body[i:i+8], i+8
		default:
			return out
		}
		out = append(out, f)
	}
	return out
}

// Float32 reads a fixed-width field left raw by ParseFields as a number.
func Float32(f Field) float64 {
	if len(f.Data) != 4 {
		return 0
	}
	return float64(math.Float32frombits(binary.LittleEndian.Uint32(f.Data)))
}

// PackedFloat32 decodes a packed repeated float. A trailing partial element is
// dropped rather than treated as an error: a torn read of a live ring is better
// served by the points that did arrive.
func PackedFloat32(b []byte) []float64 {
	out := make([]float64, 0, len(b)/4)
	for i := 0; i+4 <= len(b); i += 4 {
		out = append(out, float64(math.Float32frombits(binary.LittleEndian.Uint32(b[i:]))))
	}
	return out
}
