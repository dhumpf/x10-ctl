package cloud

// The HTTP cloud API cloud_cli calls, served on the robot itself.
//
// A robot configured to reach its cloud locally hits this, so every answer is
// local and there is no runtime dependency on anything else.
//
// Two answers come from a per-robot data file this program reads, not built at
// runtime:
//
//   - get_mqtt_info carries the broker endpoint and CA the robot connects with.
//     Only the endpoint and the CA are set to ours; the rest is served as-is.
//   - get_product_data_point is the datapoint table. dp > 158 has no synthetic
//     form, so the stored table is the only good answer.
//
// That data file is device-specific, kept mode 600 on the robot, and is not in
// this repo (see .gitignore).

import (
	"encoding/json"
	"fmt"
	"log"
	"net/http"
	"os"
	"path/filepath"
	"slices"
	"strings"
	"time"
)

// Fields rewritten in the served get_mqtt_info answer.
var (
	apiEndpointKeys = []string{"endpoint_addr", "endpoint", "host", "mqtt_host", "mqtt_addr", "mqtt_endpoint"}
	apiPortKeys     = []string{"port", "mqtt_port"}
	apiCAKeys       = []string{"aws_root_ca1_pem", "root_ca_pem", "ca_pem", "root_ca", "ca", "ca_crt"}
)

const (
	apiMQTTInfoFile   = "equipment_devicemanage_get_mqtt_info.json"
	apiDataPointFile  = "equipment_things_get_product_data_point.json"
	apiExcel2YAMLFile = "excel2yaml.yml"
)

type CloudAPI struct {
	dir        string // captured payloads
	brokerHost string // what endpoint_addr becomes
	brokerPort int
	caFile     string // our broker certificate, handed over as the CA
}

// apiOK is the envelope the robot accepted for get_mqtt_info - 286 retries
// stopped dead on this shape - so everything unremarkable uses it.
func apiOK(data any) map[string]any {
	return map[string]any{"code": 0, "msg": "success", "data": data}
}

// apiOKBang is the same with the msg the captured answers actually carry.
func apiOKBang(data any) map[string]any {
	return map[string]any{"code": 0, "msg": "success!", "data": data}
}

func (a *CloudAPI) handler() http.Handler {
	mux := http.NewServeMux()
	mux.HandleFunc("/equipment/devicemanage/get_mqtt_info", a.mqttInfo)
	mux.HandleFunc("/equipment/things/get_product_data_point", a.dataPoints)
	mux.HandleFunc("/equipment/agreement/get_device_point_switch", a.pointSwitch)
	mux.HandleFunc("/equipment/devicemanage/update_info", a.updateInfo)
	mux.HandleFunc("/mp/excel2yaml.yml", a.excel2yaml)
	mux.HandleFunc("/", a.generic)
	return mux
}

// readReq pulls the JSON body, tolerating an empty or unparsable one the way
// A decode error must not cost the robot its answer.
func readReq(r *http.Request) map[string]any {
	var m map[string]any
	body := http.MaxBytesReader(nil, r.Body, 1<<20)
	if err := json.NewDecoder(body).Decode(&m); err != nil || m == nil {
		return map[string]any{}
	}
	return m
}

func strField(m map[string]any, key string) string {
	if v, ok := m[key].(string); ok {
		return v
	}
	return ""
}

// deviceSN prefers what the robot put in the body, then its header.
func deviceSN(r *http.Request, req map[string]any) string {
	if sn := strField(req, "device_sn"); sn != "" {
		return sn
	}
	return r.Header.Get("device-sn")
}

func (a *CloudAPI) path(name string) string { return filepath.Join(a.dir, name) }

// rewriteMQTTInfo points a genuine get_mqtt_info response at our broker and our
// CA. A parse failure is an error, not a pass-through: the capture is the only
// source here, so un-rewritten bytes still carry Anker's endpoint and CA. Fail
// closed and let mqttInfo 503 - a truncated file must not send the robot to
// Anker under a success code.
func (a *CloudAPI) rewriteMQTTInfo(raw []byte, caPEM string) ([]byte, []string, error) {
	var doc any
	if err := json.Unmarshal(raw, &doc); err != nil {
		return nil, nil, fmt.Errorf("capture is not valid JSON: %w", err)
	}
	var changed []string
	var walk func(node any)
	walk = func(node any) {
		switch n := node.(type) {
		case map[string]any:
			for k, v := range n {
				s, isStr := v.(string)
				switch {
				case isStr && slices.Contains(apiEndpointKeys, k):
					n[k] = a.brokerHost
					changed = append(changed, k)
				case slices.Contains(apiPortKeys, k) && isNumber(v):
					n[k] = a.brokerPort
					changed = append(changed, k)
				case isStr && slices.Contains(apiCAKeys, k):
					n[k] = caPEM
					changed = append(changed, k)
				case isStr && strings.HasPrefix(s, "ssl://"):
					n[k] = fmt.Sprintf("ssl://%s:%d", a.brokerHost, a.brokerPort)
					changed = append(changed, k)
				default:
					walk(v)
				}
			}
		case []any:
			for _, v := range n {
				walk(v)
			}
		}
	}
	walk(doc)
	out, err := json.Marshal(doc)
	if err != nil {
		return raw, nil, err
	}
	return out, changed, nil
}

func isNumber(v any) bool {
	_, ok := v.(float64) // encoding/json decodes every number as float64
	return ok
}

// mqttInfo replays the capture with the endpoint and CA swapped for ours.
//
// There is deliberately no synthetic fallback. Credentials we generate are
// undecryptable to the robot, and handing it those looks like success while it
// quietly never connects; a 503 makes it retry against a broker it can still
// reach with its cached props.
func (a *CloudAPI) mqttInfo(w http.ResponseWriter, r *http.Request) {
	raw, err := os.ReadFile(a.path(apiMQTTInfoFile))
	if err != nil {
		log.Printf("cloudapi: get_mqtt_info has no capture at %s: %v", a.path(apiMQTTInfoFile), err)
		writeJSON(w, http.StatusServiceUnavailable,
			map[string]any{"code": 1, "msg": "no get_mqtt_info capture on this device"})
		return
	}
	caPEM, err := os.ReadFile(a.caFile)
	if err != nil {
		log.Printf("cloudapi: get_mqtt_info cannot read the broker cert %s: %v", a.caFile, err)
		writeJSON(w, http.StatusServiceUnavailable,
			map[string]any{"code": 1, "msg": "no broker certificate to hand over"})
		return
	}
	body, changed, err := a.rewriteMQTTInfo(raw, string(caPEM))
	if err != nil {
		log.Printf("cloudapi: get_mqtt_info rewrite failed: %v", err)
		writeJSON(w, http.StatusServiceUnavailable, map[string]any{"code": 1, "msg": "rewrite failed"})
		return
	}
	log.Printf("cloudapi: get_mqtt_info -> %s:%d, rewrote %v", a.brokerHost, a.brokerPort, changed)
	w.Header().Set("Content-Type", "application/json; charset=utf-8")
	w.WriteHeader(http.StatusOK)
	_, _ = w.Write(body)
}

// dataPoints replays the captured table. dp > 158 in the reconstruction were
// never accepted, so a missing capture is reported rather than papered over.
func (a *CloudAPI) dataPoints(w http.ResponseWriter, r *http.Request) {
	raw, err := os.ReadFile(a.path(apiDataPointFile))
	if err != nil {
		log.Printf("cloudapi: get_product_data_point has no capture: %v", err)
		writeJSON(w, http.StatusServiceUnavailable,
			map[string]any{"code": 1, "msg": "no get_product_data_point capture on this device"})
		return
	}
	log.Printf("cloudapi: get_product_data_point -> capture, %dB", len(raw))
	w.Header().Set("Content-Type", "application/json; charset=utf-8")
	w.WriteHeader(http.StatusOK)
	_, _ = w.Write(raw)
}

// pointSwitch: the real cloud answers an empty list for this device, and empty
// is also the safe answer - anything in it is the cloud switching datapoints
// off behind our back.
func (a *CloudAPI) pointSwitch(w http.ResponseWriter, r *http.Request) {
	_ = readReq(r)
	log.Print("cloudapi: get_device_point_switch -> empty param list")
	writeJSON(w, http.StatusOK, apiOKBang(map[string]any{"param": []any{}}))
}

// updateInfo is where the robot volunteers its SSID beside its serial. Ack and
// drop it: the point of running our own cloud is that this stops leaving the
// house. Counted in the log, never stored, never echoed.
func (a *CloudAPI) updateInfo(w http.ResponseWriter, r *http.Request) {
	req := readReq(r)
	var dropped []string
	for _, k := range []string{"wifi_ssid", "wifi_mac", "ip"} {
		if _, ok := req[k]; ok {
			dropped = append(dropped, k)
		}
	}
	if len(dropped) > 0 {
		log.Printf("cloudapi: update_info -> discarding %s", strings.Join(dropped, ", "))
	}
	writeJSON(w, http.StatusOK, apiOK(map[string]any{}))
}

// excel2yaml is the cloud-side cleaning parameter table, fetched once per
// session: every room_type x flooring_type x trash_type combination, none of
// which is in user_config.yaml. Without it the robot has no defaults to reason
// from. Served verbatim from the capture.
func (a *CloudAPI) excel2yaml(w http.ResponseWriter, r *http.Request) {
	raw, err := os.ReadFile(a.path(apiExcel2YAMLFile))
	if err != nil {
		log.Printf("cloudapi: excel2yaml.yml requested but missing: %v", err)
		http.Error(w, "not found", http.StatusNotFound)
		return
	}
	log.Printf("cloudapi: excel2yaml.yml -> %dB", len(raw))
	w.Header().Set("Content-Type", "application/x-yaml")
	w.WriteHeader(http.StatusOK)
	_, _ = w.Write(raw)
}

// generic is the catch-all. get_up_token lands here, which is what the robot
// has always been told: an empty data object, no upload token, so nothing is
// ever uploaded anywhere.
func (a *CloudAPI) generic(w http.ResponseWriter, r *http.Request) {
	_ = readReq(r)
	log.Printf("cloudapi: generic success for %s %s", r.Method, r.URL.Path)
	writeJSON(w, http.StatusOK, apiOK(map[string]any{}))
}

// serveCloudAPI runs the API on its own listener. Separate from the UI server
// because the robot reaches this one by IP:80 when configured locally, and the two have
// nothing to say to each other.
func ServeCloudAPI(a *CloudAPI, addr string, stop <-chan struct{}) {
	srv := &http.Server{
		Addr:              addr,
		Handler:           a.handler(),
		ReadHeaderTimeout: 5 * time.Second,
	}
	go func() {
		<-stop
		_ = srv.Close()
	}()
	go func() {
		log.Printf("cloudapi: serving the robot's cloud on %s from %s", addr, a.dir)
		if err := srv.ListenAndServe(); err != nil && err != http.ErrServerClosed {
			log.Printf("cloudapi: listen %s: %v", addr, err)
		}
	}()
}

// NewCloudAPI builds the onboarding API server. dir holds the captured
// payloads; brokerHost:brokerPort is advertised in get_mqtt_info; caFile is the
// broker certificate handed over as the CA.
func NewCloudAPI(dir, brokerHost string, brokerPort int, caFile string) *CloudAPI {
	return &CloudAPI{dir: dir, brokerHost: brokerHost, brokerPort: brokerPort, caFile: caFile}
}

func writeJSON(w http.ResponseWriter, code int, v any) {
	w.Header().Set("Content-Type", "application/json")
	w.WriteHeader(code)
	_ = json.NewEncoder(w).Encode(v)
}
