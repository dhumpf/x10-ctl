package app

// Channel addressing and the protobuf subset needed to read an mds record.
// Portable half of the mds support: no syscalls, so it builds off the robot
// and is unit testable on the Mac.

import (
	"encoding/binary"
)

const (
	// MurmurHash64A, the seed mds_pub uses to turn a channel name into a
	// channel id. The low 32 bits of that id are the SysV key.
	mdsSeed = 0xc70f6907
	mdsM    = 0xc6a4a7935bd1e995
	mdsR    = 47
)

// mdsChannelKey is the SysV key for a channel name.
func mdsChannelKey(name string) uint32 {
	return uint32(mdsChannelID(name))
}

func mdsChannelID(name string) uint64 {
	b := []byte(name)
	h := uint64(mdsSeed) ^ (uint64(len(b)) * mdsM)
	n := len(b) / 8 * 8
	for i := 0; i < n; i += 8 {
		k := binary.LittleEndian.Uint64(b[i : i+8])
		k *= mdsM
		k ^= k >> mdsR
		k *= mdsM
		h ^= k
		h *= mdsM
	}
	if rest := b[n:]; len(rest) > 0 {
		for i, c := range rest {
			h ^= uint64(c) << (8 * uint(i))
		}
		h *= mdsM
	}
	h ^= h >> mdsR
	h *= mdsM
	h ^= h >> mdsR
	return h
}
