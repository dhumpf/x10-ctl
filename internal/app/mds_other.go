//go:build !(linux && arm64)

package app

import "fmt"

// Off the robot there is no shared memory to attach, so every mds read fails
// and the callers fall back to whatever remote transport is configured.

func mdsNewest(channel string) ([]byte, error) {
	return nil, fmt.Errorf("mds %s: not running on the robot", channel)
}

func mdsRecent(channel string, _ int) ([][]byte, error) {
	return nil, fmt.Errorf("mds %s: not running on the robot", channel)
}

func mdsAvailable(string) bool { return false }

func readPathPoints() ([]Point, int, error) {
	return nil, 0, fmt.Errorf("path buffer: not running on the robot")
}
