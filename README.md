# mock_vehicle_server

Standalone mock servers for GCS testing:

- **Vehicle** — MAVLink UDP telemetry / commands
- **Radio** — Silvus StreamScape HTTP API + TLV UDP RSSI/temperature reports (§§5–6)

They run independently so you can exercise radio UI without the vehicle mock (or vice versa).

## Run vehicle mock

```bash
dart run bin/mock_vehicle_server.dart
```

Listens on UDP `7000`, sends to GCS on `7500` (see `lib/config/server_config.dart`).

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
