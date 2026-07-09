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

/// Generates dynamic radio metrics that stay within threshold bands for the
/// active simulation state.
class RadioSimulator {
  RadioSimulator({this.autoCycleSeconds = 0});

  /// When > 0, automatically rotates states every N seconds.
  final int autoCycleSeconds;

  RadioSimulationState _state = RadioSimulationState.normal;
  int _sequence = 0;
  int _tick = 0;
  final _rng = math.Random(42);

  RadioSimulationState get state => _state;

  bool get isStreaming => _state != RadioSimulationState.disconnected;

  void setState(RadioSimulationState state) {
    _state = state;
    print('[RADIO] Simulation state → ${state.label}');
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

  /// Apply small jitter so streamed values look live while staying in-band.
  SimulatedRadioSample nextSample() {
    if (!isStreaming) {
      return const SimulatedRadioSample.streamingDisabled();
    }

    final base = currentTargets();
    final jitter = _rng.nextInt(3) - 1; // -1, 0, +1 dBm

    _sequence++;
    return SimulatedRadioSample(
      antenna1Dbm: base.antenna1Dbm + jitter,
      antenna2Dbm: base.antenna2Dbm + jitter,
      antenna3Dbm: base.antenna3Dbm + jitter,
      antenna4Dbm: base.antenna4Dbm + jitter,
      noiseDbm: base.noiseDbm + (jitter ~/ 2),
      syncSignal: base.syncSignal + jitter,
      syncNoise: base.syncNoise,
      nodeId: ServerConfig.radioNodeId,
      sequenceNumber: _sequence,
      temperatureCelsius: base.temperatureCelsius + (jitter ~/ 2),
      maxTemperatureCelsius: base.maxTemperatureCelsius,
      overheatCount: base.overheatCount,
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
