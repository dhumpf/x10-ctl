package app

import "testing"

// Real net_cli_scan output. The entry number on the bssid line is the part that
// broke the first parser.
const sampleScan = `[Client] Ping: ok
[Client] Send set mode(station) ...
1) bssid:AA:BB:CC:00:11:33
   ssid:ExampleWiFi
   need_passwd:1
   channel:11
   dbm:-42
2) bssid:AA:BB:CC:00:11:34
   ssid:ExampleWiFi-guest
   need_passwd:0
   channel:6
   dbm:-71
`

func TestParseScan(t *testing.T) {
	aps := parseScan(sampleScan)
	if len(aps) != 2 {
		t.Fatalf("got %d networks, want 2", len(aps))
	}
	if aps[0].SSID != "ExampleWiFi" || aps[0].Channel != 11 || aps[0].DBm != -42 || !aps[0].Secured {
		t.Errorf("first network parsed as %+v", aps[0])
	}
	if aps[1].SSID != "ExampleWiFi-guest" || aps[1].Secured {
		t.Errorf("second network parsed as %+v", aps[1])
	}
}

// An access point with no networks in range must yield none, not one empty.
func TestParseScanEmpty(t *testing.T) {
	if got := parseScan("[Client] Ping: ok\n"); len(got) != 0 {
		t.Errorf("got %d networks from an empty scan, want 0", len(got))
	}
}
