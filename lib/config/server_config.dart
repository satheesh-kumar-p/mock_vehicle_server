/// Network and identity constants for the mock vehicle + Silvus radio server.
///
/// Radio settings mirror [RadioConfig] in scout-td0-GCS
/// `lib/core/constants/app_constants.dart` — keep both in sync.
class ServerConfig {
  const ServerConfig._();

  // ── MAVLink vehicle UDP (RadioConfig.host / mavlinkHost / port / mavlinkPort)
  static const String gcsHost = '127.0.0.1'; // RadioConfig.host
  static const String mavlinkHost = '127.0.0.1'; // RadioConfig.mavlinkHost
  static const int gcsPort = 7500; // RadioConfig.port
  static const int vehicleUdpPort = 7000; // RadioConfig.mavlinkPort

  static const int vehicleSysId = 1;
  static const int vehicleCompId = 191;
  static const int gcsSysId = 255;
  static const int gcsCompId = 190;

  static const int vehicleClockOffsetMs = -500;
  static const int mavlinkLinkId = 1;

  // ── Silvus radio (RadioConfig) ──────────────────────────────────────────
  /// GCS machine IP — TLV RSSI/temperature reports are sent here.
  static const String gcsHostSystemIp = '192.168.168.99';

  /// Silvus radio IP — GCS calls http://<this>/streamscape_api (port 80).
  static const String gcsRadioIp = '192.168.168.153';

  static const int udpStreamingPort = 9000;

  /// RSSI report period default (ms). Manual §3.12: default 10, range 10–1000.
  static const int udpReportTimeMs = 10;
  static const int rssiReportPeriodMinMs = 10;
  static const int rssiReportPeriodMaxMs = 1000;

  static const int udpTemperatureReportPeriodSec = 1;
  static const int tempReportingMinThresholdC = 70;
  static const int tempReportingMaxThresholdC = 85;

  /// Bind all interfaces so the API is reachable at [gcsRadioIp]:80.
  static const String radioHttpBindHost = '0.0.0.0';
  static const int radioHttpPort = 80;

  /// Default node id when not using §5 Table 2 sample (sample uses 1025).
  static const int radioNodeId = 1025;
}
