package cloud

import (
	"encoding/json"
	"net/http"
	"net/http/httptest"
	"os"
	"path/filepath"
	"strings"
	"testing"
)

// The shape of the real get_mqtt_info capture, minus the actual secrets. The
// two credential fields are what the robot decrypts with its device key, so the
// test that matters is that they come back byte-identical.
const fakeMQTTCapture = `{
  "code": 0,
  "msg": "success!",
  "data": {
    "device_sn": "AMP00X0E00000000",
    "thing_name": "AMP00X0E00000000",
    "certificate_id": "1089539172790151525",
    "certificate_pem": "-----BEGIN CERTIFICATE-----\nDEVICECERT\n-----END CERTIFICATE-----\n",
    "private_key": "-----BEGIN RSA PRIVATE KEY-----\nDEVICEKEY\n-----END RSA PRIVATE KEY-----\n",
    "public_key": "",
    "endpoint_addr": "mqtt.vendor.example",
    "aws_root_ca1_pem": "-----BEGIN CERTIFICATE-----\nANKERCA\n-----END CERTIFICATE-----\n",
    "origin": "",
    "country_code": "CA"
  },
  "trace_id": "e104d7c1"
}`

func testAPI(t *testing.T) *CloudAPI {
	t.Helper()
	dir := t.TempDir()
	if err := os.WriteFile(filepath.Join(dir, apiMQTTInfoFile), []byte(fakeMQTTCapture), 0o600); err != nil {
		t.Fatal(err)
	}
	ca := filepath.Join(dir, "broker.crt")
	if err := os.WriteFile(ca, []byte("-----BEGIN CERTIFICATE-----\nOURBROKER\n-----END CERTIFICATE-----\n"), 0o644); err != nil {
		t.Fatal(err)
	}
	return &CloudAPI{dir: dir, brokerHost: "127.0.0.1", brokerPort: 8883, caFile: ca}
}

func post(t *testing.T, a *CloudAPI, path, body string) *httptest.ResponseRecorder {
	t.Helper()
	req := httptest.NewRequest(http.MethodPost, path, strings.NewReader(body))
	w := httptest.NewRecorder()
	a.handler().ServeHTTP(w, req)
	return w
}

// The device credentials must survive the rewrite untouched. Replacing them
// with a pair of ours is undecryptable to the robot, and it then never dials
// the broker at all - the failure that made the capture necessary.
func TestMQTTInfoKeepsDeviceCredentials(t *testing.T) {
	a := testAPI(t)
	w := post(t, a, "/equipment/devicemanage/get_mqtt_info", "{}")
	if w.Code != http.StatusOK {
		t.Fatalf("status %d, want 200", w.Code)
	}
	var got struct {
		Data map[string]any `json:"data"`
	}
	if err := json.Unmarshal(w.Body.Bytes(), &got); err != nil {
		t.Fatal(err)
	}
	if cert := got.Data["certificate_pem"]; !strings.Contains(cert.(string), "DEVICECERT") {
		t.Errorf("certificate_pem was rewritten: %q", cert)
	}
	if key := got.Data["private_key"]; !strings.Contains(key.(string), "DEVICEKEY") {
		t.Errorf("private_key was rewritten: %q", key)
	}
}

// The endpoint and the CA are the two things that must change, or the robot
// dials Anker and rejects our broker.
func TestMQTTInfoPointsAtOurBroker(t *testing.T) {
	a := testAPI(t)
	w := post(t, a, "/equipment/devicemanage/get_mqtt_info", "{}")
	var got struct {
		Data map[string]any `json:"data"`
	}
	if err := json.Unmarshal(w.Body.Bytes(), &got); err != nil {
		t.Fatal(err)
	}
	if ep := got.Data["endpoint_addr"]; ep != "127.0.0.1" {
		t.Errorf("endpoint_addr = %v, want 127.0.0.1", ep)
	}
	ca, _ := got.Data["aws_root_ca1_pem"].(string)
	if !strings.Contains(ca, "OURBROKER") {
		t.Errorf("aws_root_ca1_pem still carries the old CA: %q", ca)
	}
	if strings.Contains(ca, "ANKERCA") {
		t.Error("aws_root_ca1_pem was not replaced")
	}
}

// No synthetic fallback: credentials we generate look like success and leave
// the robot silently unable to connect. A missing capture must say so.
func TestMQTTInfoWithNoCaptureFailsLoudly(t *testing.T) {
	a := testAPI(t)
	if err := os.Remove(filepath.Join(a.dir, apiMQTTInfoFile)); err != nil {
		t.Fatal(err)
	}
	w := post(t, a, "/equipment/devicemanage/get_mqtt_info", "{}")
	if w.Code == http.StatusOK {
		t.Errorf("status 200 with no capture; a synthetic answer is worse than an error")
	}
}

// A truncated or otherwise unparsable capture must fail closed. The un-rewritten
// bytes still carry Anker's endpoint and CA, so a 200 here silently sends the
// robot to Anker under a success code.
func TestMQTTInfoWithCorruptCaptureFailsClosed(t *testing.T) {
	a := testAPI(t)
	half := fakeMQTTCapture[:len(fakeMQTTCapture)/2]
	if err := os.WriteFile(filepath.Join(a.dir, apiMQTTInfoFile), []byte(half), 0o600); err != nil {
		t.Fatal(err)
	}
	w := post(t, a, "/equipment/devicemanage/get_mqtt_info", "{}")
	if w.Code == http.StatusOK {
		t.Fatalf("status 200 on a corrupt capture; the robot would be handed Anker's endpoint")
	}
	if strings.Contains(w.Body.String(), "mqtt.vendor.example") {
		t.Error("Anker's endpoint leaked in the response body")
	}
}

// get_up_token has no handler by design, so it lands on the catch-all and the
// robot is told there is no upload token - which is why nothing ever uploads.
func TestUpTokenGetsEmptySuccess(t *testing.T) {
	a := testAPI(t)
	w := post(t, a, "/equipment/cloudstor/get_up_token",
		`{"file_name":"clean_record_1786053568.bin","type":1011}`)
	if w.Code != http.StatusOK {
		t.Fatalf("status %d, want 200", w.Code)
	}
	var got struct {
		Code int            `json:"code"`
		Data map[string]any `json:"data"`
	}
	if err := json.Unmarshal(w.Body.Bytes(), &got); err != nil {
		t.Fatal(err)
	}
	if got.Code != 0 {
		t.Errorf("code = %d, want 0", got.Code)
	}
	if len(got.Data) != 0 {
		t.Errorf("data = %v, want empty; a token here would start real uploads", got.Data)
	}
}

// Anything in this list is the cloud switching datapoints off behind our back.
func TestPointSwitchIsEmpty(t *testing.T) {
	a := testAPI(t)
	w := post(t, a, "/equipment/agreement/get_device_point_switch", "{}")
	var got struct {
		Data struct {
			Param []any `json:"param"`
		} `json:"data"`
	}
	if err := json.Unmarshal(w.Body.Bytes(), &got); err != nil {
		t.Fatal(err)
	}
	if len(got.Data.Param) != 0 {
		t.Errorf("param = %v, want empty", got.Data.Param)
	}
}

// The robot volunteers its SSID here. Acknowledge, drop it, never echo it.
func TestUpdateInfoDropsTheSSID(t *testing.T) {
	a := testAPI(t)
	w := post(t, a, "/equipment/devicemanage/update_info",
		`{"device_sn":"AMP00X0E00000000","wifi_ssid":"ExampleIoT","main_sw_version":"v3.4.85"}`)
	if w.Code != http.StatusOK {
		t.Fatalf("status %d, want 200", w.Code)
	}
	if strings.Contains(w.Body.String(), "ExampleIoT") {
		t.Error("the response echoed the SSID back")
	}
}

// A body that is not JSON must still get an answer; the server tolerates it and
// a decode error must not cost the robot its session.
func TestGarbageBodyStillAnswers(t *testing.T) {
	a := testAPI(t)
	w := post(t, a, "/equipment/devicemanage/update_info", "not json at all")
	if w.Code != http.StatusOK {
		t.Errorf("status %d on a bad body, want 200", w.Code)
	}
}
