/// SILVUS 4200 threshold bands from silvus_thresholds.xlsx.
///
/// Columns: Degraded (amber) | Healthy (green) | Fault (red)
class RadioThresholds {
  const RadioThresholds._();

  // RSSI / Remote RSSI (dBm)
  static const int rssiHealthyAboveDbm = -85;
  static const int rssiFaultBelowDbm = -95;

  // SNR (dB)
  static const double snrHealthyAboveDb = 10;
  static const double snrFaultBelowDb = 5;

  // Noise (dBm) — matches RadioHealthEvaluator in scout-td0-GCS
  static const int noiseHealthyBelowDbm = -90;
  static const int noiseFaultAboveDbm = -95;

  // Temperature (°C)
  static const int tempHealthyMinC = -40;
  static const int tempHealthyMaxC = 85;
}

/// Simulated metric targets for each UI verification state.
class RadioMetricTargets {
  const RadioMetricTargets({
    required this.antenna1Dbm,
    required this.antenna2Dbm,
    required this.antenna3Dbm,
    required this.antenna4Dbm,
    required this.noiseDbm,
    required this.syncSignal,
    required this.syncNoise,
    required this.temperatureCelsius,
    required this.maxTemperatureCelsius,
    required this.overheatCount,
  });

  /// §5 Table 2 sample values (manual exact RSSI example).
  static const normal = RadioMetricTargets(
    antenna1Dbm: -43,
    antenna2Dbm: -31,
    antenna3Dbm: -28,
    antenna4Dbm: -66,
    noiseDbm: -190,
    syncSignal: 8604568,
    syncNoise: 8861322,
    temperatureCelsius: 45,
    maxTemperatureCelsius: 55,
    overheatCount: 0,
  );

  /// Metrics in degraded / amber bands.
  static const warning = RadioMetricTargets(
    antenna1Dbm: -88,
    antenna2Dbm: -87,
    antenna3Dbm: -89,
    antenna4Dbm: -88,
    noiseDbm: -90,
    syncSignal: 140,
    syncNoise: 125,
    temperatureCelsius: 82,
    maxTemperatureCelsius: 84,
    overheatCount: 1,
  );

  /// Metrics in fault / red bands.
  static const critical = RadioMetricTargets(
    antenna1Dbm: -98,
    antenna2Dbm: -97,
    antenna3Dbm: -99,
    antenna4Dbm: -98,
    noiseDbm: -80,
    syncSignal: 110,
    syncNoise: 130,
    temperatureCelsius: 92,
    maxTemperatureCelsius: 95,
    overheatCount: 3,
  );

  final int antenna1Dbm;
  final int antenna2Dbm;
  final int antenna3Dbm;
  final int antenna4Dbm;
  final int noiseDbm;
  final int syncSignal;
  final int syncNoise;
  final int temperatureCelsius;
  final int maxTemperatureCelsius;
  final int overheatCount;

  /// Encode signed dBm into MAVLink RADIO_STATUS uint8 (0–254, 255 invalid).
  int encodeRssiDbm(int dbm) => (dbm + 120).clamp(0, 254);

  int get mavlinkRssi => encodeRssiDbm(antenna1Dbm);
  int get mavlinkRemRssi => encodeRssiDbm(antenna2Dbm);
  int get mavlinkNoise => encodeRssiDbm(noiseDbm);
  int get mavlinkRemNoise => encodeRssiDbm(noiseDbm - 2);
}
