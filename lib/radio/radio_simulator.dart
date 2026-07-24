import 'dart:math' as math;

import '../config/server_config.dart';
import 'radio_thresholds.dart';

enum RadioSimulationState {
  normal,
  warning,
  critical,
  disconnected,
}

extension RadioSimulationStateLabel on RadioSimulationState {
  String get label => switch (this) {
        RadioSimulationState.normal => 'normal',
        RadioSimulationState.warning => 'warning',
        RadioSimulationState.critical => 'critical',
        RadioSimulationState.disconnected => 'disconnected',
      };

  static RadioSimulationState? tryParse(String value) {
    return switch (value.toLowerCase().trim()) {
      'normal' || 'healthy' || 'green' => RadioSimulationState.normal,
      'warning' || 'degraded' || 'amber' => RadioSimulationState.warning,
      'critical' || 'fault' || 'red' => RadioSimulationState.critical,
      'disconnected' || 'offline' || 'none' => RadioSimulationState.disconnected,
      _ => null,
    };
  }
}

/// Generates radio metrics for TLV streaming (§§5–6).
///
/// When [useManualSample] is true (default), RSSI samples match §5 Table 2
/// exactly (no jitter). When false, values follow [RadioSimulationState] bands
/// with small jitter for live UI verification.
class RadioSimulator {
  RadioSimulator({
    this.autoCycleSeconds = 0,
    this.useManualSample = true,
    this.keepaliveNoTraffic = false,
    int? nodeId,
  }) : nodeId = nodeId ?? ServerConfig.radioNodeId;

  /// When > 0, automatically rotates states every N seconds.
  final int autoCycleSeconds;

  /// Emit exact §5 Table 2 values (sequence fixed at 2333 when true).
  bool useManualSample;

  /// When true, RSSI fields are all 999 (manual no-traffic keepalive).
  bool keepaliveNoTraffic;

  final int nodeId;

  RadioSimulationState _state = RadioSimulationState.normal;
  int _sequence = 0;
  int _tick = 0;
  final _rng = math.Random(42);

  int _maxTemperatureCelsius = RadioMetricTargets.normal.maxTemperatureCelsius;
  int _overheatCount = 0;
  bool _wasOverheating = false;
  int _tempMaxThresholdC = ServerConfig.tempReportingMaxThresholdC;

  RadioSimulationState get state => _state;

  bool get isStreaming => _state != RadioSimulationState.disconnected;

  int get overheatCount => _overheatCount;

  int get maxTemperatureCelsius => _maxTemperatureCelsius;

  void setState(RadioSimulationState state) {
    _state = state;
    print('[RADIO] Simulation state → ${state.label}');
  }

  void setTempMaxThreshold(int celsius) {
    _tempMaxThresholdC = celsius;
  }

  void tick() {
    _tick++;
    if (autoCycleSeconds > 0 && _tick % autoCycleSeconds == 0) {
      final values = RadioSimulationState.values;
      final next = values[(_state.index + 1) % values.length];
      setState(next);
    }
  }

  RadioMetricTargets currentTargets() {
    return switch (_state) {
      RadioSimulationState.normal => RadioMetricTargets.normal,
      RadioSimulationState.warning => RadioMetricTargets.warning,
      RadioSimulationState.critical => RadioMetricTargets.critical,
      RadioSimulationState.disconnected => RadioMetricTargets.normal,
    };
  }

  /// Next sequence number: increments each report, resets after 9999 (§5).
  int _nextSequence() {
    _sequence++;
    if (_sequence > 9999) {
      _sequence = 1;
    }
    return _sequence;
  }

  void _updateTemperatureTracking(int currentTemp) {
    if (currentTemp > _maxTemperatureCelsius) {
      _maxTemperatureCelsius = currentTemp;
    }
    final overheating = currentTemp > _tempMaxThresholdC;
    if (overheating && !_wasOverheating) {
      _overheatCount++;
    }
    _wasOverheating = overheating;
  }

  /// Apply small jitter so streamed values look live while staying in-band.
  SimulatedRadioSample nextSample() {
    if (!isStreaming) {
      return const SimulatedRadioSample.streamingDisabled();
    }

    if (keepaliveNoTraffic) {
      final seq = useManualSample
          ? SimulatedRadioSample.manualTable2.sequenceNumber
          : _nextSequence();
      return SimulatedRadioSample.keepalive(
        nodeId: useManualSample
            ? SimulatedRadioSample.manualTable2.nodeId
            : nodeId,
        sequenceNumber: seq,
      );
    }

    if (useManualSample) {
      final sample = SimulatedRadioSample.manualTable2;
      _updateTemperatureTracking(sample.temperatureCelsius);
      return SimulatedRadioSample(
        antenna1Dbm: sample.antenna1Dbm,
        antenna2Dbm: sample.antenna2Dbm,
        antenna3Dbm: sample.antenna3Dbm,
        antenna4Dbm: sample.antenna4Dbm,
        noiseDbm: sample.noiseDbm,
        syncSignal: sample.syncSignal,
        syncNoise: sample.syncNoise,
        nodeId: sample.nodeId,
        sequenceNumber: sample.sequenceNumber,
        temperatureCelsius: sample.temperatureCelsius,
        maxTemperatureCelsius: math.max(
          sample.maxTemperatureCelsius,
          _maxTemperatureCelsius,
        ),
        overheatCount: math.max(sample.overheatCount, _overheatCount),
      );
    }

    final base = currentTargets();
    final jitter = _rng.nextInt(3) - 1; // -1, 0, +1
    final temp = base.temperatureCelsius + (jitter ~/ 2);
    _updateTemperatureTracking(temp);

    return SimulatedRadioSample(
      antenna1Dbm: base.antenna1Dbm + jitter,
      antenna2Dbm: base.antenna2Dbm + jitter,
      antenna3Dbm: base.antenna3Dbm + jitter,
      antenna4Dbm: base.antenna4Dbm + jitter,
      noiseDbm: base.noiseDbm + (jitter ~/ 2),
      syncSignal: base.syncSignal + jitter,
      syncNoise: base.syncNoise,
      nodeId: nodeId,
      sequenceNumber: _nextSequence(),
      temperatureCelsius: temp,
      maxTemperatureCelsius: math.max(
        base.maxTemperatureCelsius,
        _maxTemperatureCelsius,
      ),
      overheatCount: math.max(base.overheatCount, _overheatCount),
    );
  }
}

class SimulatedRadioSample {
  const SimulatedRadioSample({
    required this.antenna1Dbm,
    required this.antenna2Dbm,
    required this.antenna3Dbm,
    required this.antenna4Dbm,
    required this.noiseDbm,
    required this.syncSignal,
    required this.syncNoise,
    required this.nodeId,
    required this.sequenceNumber,
    required this.temperatureCelsius,
    required this.maxTemperatureCelsius,
    required this.overheatCount,
  });

  /// §5 Table 2 sample values (exact manual RSSI example).
  static const manualTable2 = SimulatedRadioSample(
    antenna1Dbm: -43,
    antenna2Dbm: -31,
    antenna3Dbm: -28,
    antenna4Dbm: -66,
    noiseDbm: -190,
    syncSignal: 8604568,
    syncNoise: 8861322,
    nodeId: 1025,
    sequenceNumber: 2333,
    temperatureCelsius: 45,
    maxTemperatureCelsius: 55,
    overheatCount: 0,
  );

  const SimulatedRadioSample.streamingDisabled()
      : antenna1Dbm = 0,
        antenna2Dbm = 0,
        antenna3Dbm = 0,
        antenna4Dbm = 0,
        noiseDbm = 0,
        syncSignal = 0,
        syncNoise = 0,
        nodeId = 0,
        sequenceNumber = 0,
        temperatureCelsius = 0,
        maxTemperatureCelsius = 0,
        overheatCount = 0;

  /// §5 no-traffic keepalive: all RSSI metric fields set to 999.
  const SimulatedRadioSample.keepalive({
    required this.nodeId,
    required this.sequenceNumber,
  })  : antenna1Dbm = 999,
        antenna2Dbm = 999,
        antenna3Dbm = 999,
        antenna4Dbm = 999,
        noiseDbm = 999,
        syncSignal = 999,
        syncNoise = 999,
        temperatureCelsius = 0,
        maxTemperatureCelsius = 0,
        overheatCount = 0;

  final int antenna1Dbm;
  final int antenna2Dbm;
  final int antenna3Dbm;
  final int antenna4Dbm;
  final int noiseDbm;
  final int syncSignal;
  final int syncNoise;
  final int nodeId;
  final int sequenceNumber;
  final int temperatureCelsius;
  final int maxTemperatureCelsius;
  final int overheatCount;

  int encodeRssiDbm(int dbm) => (dbm + 120).clamp(0, 254);

  int get mavlinkRssi => encodeRssiDbm(antenna1Dbm);
  int get mavlinkRemRssi => encodeRssiDbm(antenna2Dbm);
  int get mavlinkNoise => encodeRssiDbm(noiseDbm);
  int get mavlinkRemNoise => encodeRssiDbm(noiseDbm - 2);
}
