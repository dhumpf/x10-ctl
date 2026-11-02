# x10-ctl

x10-ctl is a single static Go binary that gives you local control and a local web UI for a eufy Clean X10 Pro Omni (T2351) robot vacuum, without the vendor cloud. It talks to the robot's 9668 control port, and, when it runs on the robot itself, to the robot's own internal bus through the vendor CLIs that are already installed on the device. On top of that it offers a Home Assistant bridge.

## Scope

This is local control for a robot you **already have root on**. How you get root is out of scope for this project and is not covered here. If you do not control the device, this tool is not for you.

## Requirements

| Need | Notes |
|---|---|
| Go | 1.24 or newer (see `go.mod`) |
| A eufy Clean X10 Pro Omni (T2351) | On your LAN, and one you control |
| Root on that robot | Already in hand; see Scope |
| `make` | Optional, wraps the `go build` calls |

The on-device features only activate when x10-ctl is running on the robot. Anywhere else those paths are inert and only the 9668 client is used.

## Build

Bare `make` lists the targets and changes nothing.

| Command | What it does |
|---|---|
| `make setup` | Fetch Go modules |
| `make build` | Build for this host |
| `make cross` | Build `x10-ctl-arm64` for the robot (linux/arm64, `CGO_ENABLED=0`, static) |
| `make test` | Host unit tests |
| `make lint` | `go vet` |
| `make fmt` | `gofmt` the tree |
| `make clean` | Remove build outputs |

## Layout

| Package | What it holds |
|---|---|
| `cmd/x10-ctl` | Entrypoint: parses flags, wires the pieces, serves until signalled |
| `cmd/mdsdump` | Standalone probe that dumps an on-device mds channel (arm64 Linux) |
| `internal/app` | Web server, HTTP handlers, on-device control (`Local`) and the Home Assistant bridge |
| `internal/client` | The robot's LAN protocols: the TCP 9668 control client and UDP discovery |
| `internal/cloud` | The cloud MQTT control path and the onboarding API server |
| `internal/mqtt` | MQTT wire codec, an embedded broker and a client |
| `internal/dp` | Data-point vocabulary and the ModeCtrl method codes |
| `internal/pb` | Protobuf wire primitives the robot's messages are built from |

## Features

| Area | What it does |
|---|---|
| Web UI | One page on `-addr` (default `:8080`) with buttons for the commands below |
| Cleaning | Auto clean, room clean, zone clean, spot clean, carpet clean, pause, resume, stop, go home |
| Manual drive | Move and turn from the UI |
| Map | Rendered map, room list, room rename, merge, divide, zones, map revert |
| Room settings | Per-room cleaning parameters |
| Schedules | List and edit schedules, with room selection |
| Dock | Wash, dry (including dry configuration), dust collection, peak-charge window |
| Robot settings | Volume, do-not-disturb, general cleaning settings, consumables, clock, Wi-Fi |
| Sensors | Battery, lidar, pose, camera snapshot and light |
| Home Assistant | MQTT autodiscovery of one device with its entities (`-ha`) |

Commands that need a target (rooms, zones, a point) are rejected when sent without one. The UI marks the commands that physically move the robot.

### Control paths

| Path | Carries | Needs |
|---|---|---|
| 9668 client (`internal/client`) | Mode commands over TCP on the LAN | Network reach to the robot |
| MQTT (`internal/cloud`, `internal/mqtt`) | Data points 9668 cannot reach: drying, volume, beep, battery | The robot pointed at a broker you run (see MQTT) |
| On-device bus (`internal/app`) | Most commands and telemetry, via the vendor CLIs already on the robot | x10-ctl running on the robot |

Movement prefers the on-device bus when it is available. `-no-local-ctrl` forces it back onto the 9668 path. When x10-ctl runs on the robot, the on-device bus already covers every UI control, so the MQTT path is optional (`-mqtt off`).

## Running

```sh
./x10-ctl -addr :8080
```

The UI is at `http://<host>:8080/`.

| Flag | Default | Purpose |
|---|---|---|
| `-addr` | `:8080` | Listen address for the web UI |
| `-host` | empty | Robot IP (default: learned from the LAN announcement) |
| `-port` | `9668` | Robot control port |
| `-timeout` | `6s` | Per-request robot timeout |
| `-no-local-ctrl` | off | Do not route movement through the on-device bus |
| `-ha` | off | Publish Home Assistant autodiscovery |
| `-ha-broker` | empty | Your Home Assistant MQTT broker, `host:port` |
| `-ha-user`, `-ha-pass` | empty | Broker credentials, if it wants them |
| `-ha-ca` | empty | CA bundle for that broker |
| `-ha-insecure` | off | Skip broker certificate verification |
| `-ha-plain` | off | Connect without TLS |
| `-ha-interval` | `10s` | How often to look for changed state |
| `-ap-fallback` | empty | SSID of an access point to raise when `wlan0` has no address |
| `-ap-pass` | empty | Password for that access point |
| `-ap-grace` | `90s` | How long `wlan0` may be down before the access point goes up |

Run `./x10-ctl -h` for the complete flag list.

### Example: Home Assistant

```sh
./x10-ctl -ha -ha-broker ha-broker.example.lan:1883 -ha-plain
```

`ha-broker.example.lan` is a placeholder; use your own broker.

## MQTT

Some of the robot's state and commands (drying, volume, beep, and battery that the 9668 path structurally cannot reach) travel over MQTT, which on a stock robot means the vendor cloud. To keep them local, x10-ctl can be the broker the robot talks to on your own LAN. `-mqtt` selects the path; the default is `off`.

| Mode | What it does | When |
|---|---|---|
| `off` | No MQTT. Those data points are unavailable; 9668 and the on-device bus still work | The default, and all you need when running on the robot |
| `internal` | x10-ctl runs an embedded broker and serves the robot's cloud API (`-api-addr`), so the robot talks to your LAN instead of the vendor | You want the MQTT-only features without a separate broker |
| `external` | x10-ctl joins a broker you already run | You already operate a broker for other things |

```sh
./x10-ctl -mqtt internal -api-addr :80
./x10-ctl -mqtt external -mqtt-broker broker.example.lan:8883 -mqtt-ca ca.crt
```

For any of this, a robot you administer has to be configured to use your broker and to trust it. That configuration is specific to your device and out of scope for this project; x10-ctl does not do it for you. `-mqtt internal` generates a self-signed broker certificate on first run (`-mqtt-cert`/`-mqtt-key`).

Relevant flags: `-mqtt-listen` (embedded broker address, default `:8883`), `-mqtt-cert`/`-mqtt-key`, `-mqtt-broker`, `-mqtt-ca`, `-mqtt-insecure`, `-api-addr`, `-api-dir`. See `./x10-ctl -h`.

## HTTP API

The web UI is a client of a small JSON API on `-addr`. A few routes:

| Method | Path | Notes |
|---|---|---|
| GET | `/api/status` | Identity, battery and bus state. Moves nothing |
| POST | `/api/command` | `{"cmd":"gohome"}` and the other named commands |
| GET | `/api/map` | Map document (Valetudo map format) |
| GET | `/api/rooms` | Room list |
| POST | `/api/roomclean` | Clean selected rooms |

The full route table is in `internal/app/main.go`.

## Licence

GPL-3.0. See `LICENSE`.

## Attribution

x10-ctl's web UI map uses the [Valetudo](https://github.com/Hypfer/Valetudo) map document format. Valetudo is AGPL-3.0; none of its code is included in this repository.
