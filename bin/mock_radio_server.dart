import 'dart:io';

import 'package:mock_vehicle_server/radio/radio_mock_server.dart';
import 'package:mock_vehicle_server/radio/radio_simulator.dart';

/// Independent Silvus radio mock (HTTP + TLV UDP).
///
/// Default RSSI payload matches StreamCaster API Manual §5 Table 2.
///
/// Usage:
///   dart run bin/mock_radio_server.dart
///   dart run bin/mock_radio_server.dart --manual-sample
///   dart run bin/mock_radio_server.dart --dynamic --state=warning
///   dart run bin/mock_radio_server.dart --keepalive
///   dart run bin/mock_radio_server.dart --auto-cycle=30
Future<void> main(List<String> args) async {
  final options = _parseArgs(args);
  if (options == null) {
    exit(64);
  }

  final server = RadioMockServer(
    initialState: options.state,
    autoCycleSeconds: options.autoCycleSeconds,
    useManualSample: options.useManualSample,
    keepaliveNoTraffic: options.keepaliveNoTraffic,
  );

  await server.start();
  print('Radio mock running. Press Ctrl+C to stop.\n');
  await Future<void>.delayed(const Duration(days: 365));
}

class _Options {
  const _Options({
    required this.state,
    required this.autoCycleSeconds,
    required this.useManualSample,
    required this.keepaliveNoTraffic,
  });

  final RadioSimulationState state;
  final int autoCycleSeconds;
  final bool useManualSample;
  final bool keepaliveNoTraffic;
}

_Options? _parseArgs(List<String> args) {
  var state = RadioSimulationState.normal;
  var autoCycleSeconds = 0;
  var useManualSample = true;
  var keepaliveNoTraffic = false;

  for (final arg in args) {
    if (arg == '--help' || arg == '-h') {
      _printUsage();
      return null;
    }

    if (arg == '--manual-sample') {
      useManualSample = true;
      continue;
    }

    if (arg == '--dynamic') {
      useManualSample = false;
      continue;
    }

    if (arg == '--keepalive') {
      keepaliveNoTraffic = true;
      continue;
    }

    if (arg.startsWith('--state=')) {
      final parsed = RadioSimulationStateLabel.tryParse(
        arg.substring('--state='.length),
      );
      if (parsed == null) {
        stderr.writeln(
          'Invalid --state. Use: normal, warning, critical, disconnected',
        );
        _printUsage();
        return null;
      }
      state = parsed;
      continue;
    }

    if (arg.startsWith('--auto-cycle=')) {
      final value = int.tryParse(arg.substring('--auto-cycle='.length));
      if (value == null || value < 0) {
        stderr.writeln(
          'Invalid --auto-cycle. Use a non-negative integer (seconds).',
        );
        _printUsage();
        return null;
      }
      autoCycleSeconds = value;
      continue;
    }

    stderr.writeln('Unknown argument: $arg');
    _printUsage();
    return null;
  }

  return _Options(
    state: state,
    autoCycleSeconds: autoCycleSeconds,
    useManualSample: useManualSample,
    keepaliveNoTraffic: keepaliveNoTraffic,
  );
}

void _printUsage() {
  stdout.writeln('''
Mock Silvus radio server (HTTP StreamScape + TLV UDP §§5–6).

Usage:
  dart run bin/mock_radio_server.dart [options]

Options:
  --manual-sample      Emit exact §5 Table 2 RSSI bytes (default)
  --dynamic            Jittered values from --state bands instead of Table 2
  --keepalive          RSSI fields all 999 (manual no-traffic keepalive)
  --state=<name>       Initial state: normal, warning, critical, disconnected
  --auto-cycle=<sec>   Rotate states every N seconds (0 = disabled)
  -h, --help           Show this help
''');
}
