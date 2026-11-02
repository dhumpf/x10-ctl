//go:build !(linux && arm64)

package app

// The camera lives in a SysV segment on the robot and is reached with arm64
// syscall numbers, so everywhere else it simply does not exist. Keeping the
// stub means the rest of the program, and the tests, build on a laptop.

import "fmt"

type frame struct {
	W, H  int
	Pix   []byte
	Stamp float64
}

func grabFrame() (*frame, error) {
	return nil, fmt.Errorf("camera is only readable on the robot (linux/arm64)")
}

func cameraAvailable() bool { return false }
