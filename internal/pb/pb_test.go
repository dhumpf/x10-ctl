package pb

import "testing"

func TestVarintRoundTrip(t *testing.T) {
	for _, n := range []uint64{0, 1, 127, 128, 300, 16383, 16384, 1 << 31, 1<<64 - 1} {
		v, i, ok := ReadVarint(Varint(n), 0)
		if !ok || v != n || i != len(Varint(n)) {
			t.Fatalf("varint %d: got %d ok=%v consumed=%d", n, v, ok, i)
		}
	}
}
