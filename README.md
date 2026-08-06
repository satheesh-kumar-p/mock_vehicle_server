# mock_vehicle_server

Standalone mock servers for GCS testing:

- **Vehicle** — MAVLink UDP telemetry / commands
- **Radio** — Silvus StreamScape HTTP API + TLV UDP RSSI/temperature reports (§§5–6)

They run independently so you can exercise radio UI without the vehicle mock (or vice versa).

## Run vehicle mock

```bash
# Default: all health checks pass (Polaris Connect success)
dart run bin/mock_vehicle_server.dart

# Lock health to a Connect Flow POST outcome
dart run bin/mock_vehicle_server.dart --state=success
dart run bin/mock_vehicle_server.dart --state=partial
dart run bin/mock_vehicle_server.dart --state=failure

# Legacy rotating faults / modes every 30 s
dart run bin/mock_vehicle_server.dart --state=cycle
```

Listens on UDP `7000`, sends to GCS on `7500` (see `lib/config/server_config.dart`).

On start it also launches a GStreamer H.264 RTP test pattern to `udp://127.0.0.1:5000`
(`gst-launch-1.0` must be on `PATH`).

### Health scenarios (Polaris Connect Flow)

GCS maps `UGV_SYSTEM_INFO` into Battery / Motor / Sensor POST rows. Use `--state=` to hold one outcome:

| `--state` | Battery | Motor | Sensor | GCS header |
|-----------|---------|-------|--------|------------|
| `success` (default) | pass | pass | pass | Connection Successful! Please wait redirecting …… |
| `partial` | pass | **fail** | pass | Error Detected, Please Contact Technical Support for resolution |
| `failure` | **fail** | **fail** | **fail** | Error Detected, Please Contact Technical Support for resolution |
| `cycle` | rotates | rotates | rotates | changes every ~5–10 s |

**GCS must use the mock network profile** or commands never reach this process:

```dart
// scout-td0-GCS/lib/core/constants/app_constants.dart
static const AppNetworkProfile profile = AppNetworkProfile.mock;
```

| Direction | Mock profile | Production profile |
|-----------|--------------|--------------------|
| GCS → vehicle | `127.0.0.1:7000` | `192.168.168.98:7000` |
| Vehicle → GCS | `127.0.0.1:7500` | bind `0.0.0.0:7500` |

With production selected, telemetry can still appear (mock → localhost:7500) while SET_HOME / other commands go to the real vehicle IP and never hit the mock.

### ICD v1.4 MAVLink compliance (§§5.1.6.2–5.1.7.2)

Dialect: `scout_mavlink_dart` **v1.4.0** (same package as GCS).

| ICD | Message | ID / cmd | Rate / notes |
|-----|---------|----------|--------------|
| 5.1.6.2.1 | COMP_HEARTBEAT (`HEARTBEAT`) | 0 | 1 Hz outbound |
| 5.1.6.2.2 | GCS_HEARTBEAT | 0 | inbound |
| 5.1.6.2.3–.4 | TIMESYNC | 111 | reply when `tc1==0` |
| 5.1.6.2.5 | MODE | cmd **176** | handled |
| 5.1.6.2.6 | ARM/DISARM | cmd **400** | handled |
| 5.1.6.2.7 | DRIVE_MODE | cmd **31900** | handled |
| 5.1.6.2.8 | MANUAL_CONTROL | 69 | log only |
| 5.1.6.2.9 | LIGHT | cmd **31901** | handled |
| 5.1.6.2.10 | CAMERA_MARKER | cmd **31902** | handled |
| 5.1.6.2.11 | REMOTE_ESTOP | cmd **31904** | handled |
| 5.1.6.2.12 | SET_HOME | cmd **179** | latch GPS → home |
| 5.1.6.2.13 | OVERRIDE_SAFETY | cmd **31903** | sets arm override |
| 5.1.6.2.14 | RTH | cmd **20** | accepted (no path follow) |
| 5.1.6.2.15 | STATUSTEXT | 253 | not emitted (event-only) |
| 5.1.7.2.1 | UGV_SYSTEM_INFO | 50001 | 1 Hz, 55-byte payload |
| 5.1.7.2.2 | SYSTEM_TIME | 2 | one-shot at startup |
| 5.1.7.2.3 | GPS_RAW_INT | 24 | **10 Hz** |
| 5.1.7.2.4 | ATTITUDE | 30 | **10 Hz** |

## Run radio mock (independent)

```bash
# Default: exact §5 Table 2 RSSI TLV (109 bytes) + §6 temperature framing
dart run bin/mock_radio_server.dart

# Dynamic threshold bands instead of Table 2
dart run bin/mock_radio_server.dart --dynamic --state=warning

# No-traffic keepalive (all RSSI fields 999)
dart run bin/mock_radio_server.dart --keepalive

dart run bin/mock_radio_server.dart --auto-cycle=30
```

| Endpoint | Purpose |
|----------|---------|
| `http://<gcsRadioIp>:80/streamscape_api` | Silvus JSON-RPC (RSSI/temp reporting config) |
| `GET/POST http://0.0.0.0:80/control/radio-state` | Force state / keepalive / manual-sample flags |

Enable RSSI streaming (required before UDP RSSI reports):

```bash
curl -X POST -d '{"jsonrpc":"2.0","method":"rssi_report_enable","params":["1"],"id":1}' \
  http://192.168.168.153/streamscape_api
```

TLV reports default to `gcsHostSystemIp:udpStreamingPort` from `ServerConfig`.

## Layout

```
bin/mock_vehicle_server.dart   # MAVLink vehicle only
bin/mock_radio_server.dart     # Silvus radio only (§§5–6)
lib/radio/                     # Radio simulation stack
  radio_mock_server.dart       # Facade used by the radio entrypoint
  radio_simulator.dart
  radio_http_server.dart
  radio_udp_streamer.dart
  radio_tlv_encoder.dart       # Null-terminated TLV per §4
  radio_thresholds.dart
lib/config/server_config.dart
test/radio_tlv_encoder_test.dart
```
