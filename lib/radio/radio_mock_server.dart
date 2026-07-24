import 'dart:async';

import '../config/server_config.dart';
import 'radio_http_server.dart';
import 'radio_simulator.dart';
import 'radio_udp_streamer.dart';

/// Standalone Silvus radio mock: HTTP StreamScape API + TLV UDP streaming.
///
/// Default RSSI payload matches StreamCaster API Manual §5 Table 2 (109 bytes).
/// Run via `bin/mock_radio_server.dart`.
class RadioMockServer {
  RadioMockServer({
    RadioSimulationState initialState = RadioSimulationState.normal,
    int autoCycleSeconds = 0,
    bool useManualSample = true,
    bool keepaliveNoTraffic = false,
  })  : _simulator = RadioSimulator(
          autoCycleSeconds: autoCycleSeconds,
          useManualSample: useManualSample,
          keepaliveNoTraffic: keepaliveNoTraffic,
        ),
        _initialState = initialState;

  final RadioSimulator _simulator;
  final RadioSimulationState _initialState;

  late final RadioHttpServer _httpServer;
  late final RadioUdpStreamer _udpStreamer;
  Timer? _tickTimer;
  bool _running = false;

  RadioSimulator get simulator => _simulator;
  RadioHttpServer get httpServer => _httpServer;
  bool get isRunning => _running;

  Future<void> start() async {
    if (_running) return;

    _simulator.setState(_initialState);

    _httpServer = RadioHttpServer(simulator: _simulator);
    _udpStreamer = RadioUdpStreamer(
      simulator: _simulator,
      httpServer: _httpServer,
    );
    _httpServer.onConfigChanged = _udpStreamer.updateTimers;

    await _httpServer.start();
    await _udpStreamer.start();
    _udpStreamer.updateTimers();

    _tickTimer = Timer.periodic(const Duration(seconds: 1), (_) {
      _simulator.tick();
    });

    _running = true;
    _printBanner();
  }

  Future<void> stop() async {
    if (!_running) return;
    _tickTimer?.cancel();
    _tickTimer = null;
    await _udpStreamer.stop();
    await _httpServer.stop();
    _running = false;
  }

  void _printBanner() {
    final mode = _simulator.useManualSample
        ? '§5 Table 2 sample'
        : 'dynamic (${_simulator.state.label})';
    print('╔══════════════════════════════════════╗');
    print('║    Mock Silvus Radio Server          ║');
    print('╠══════════════════════════════════════╣');
    print(
      '║ Silvus HTTP     : ${ServerConfig.gcsRadioIp}:${ServerConfig.radioHttpPort} ║',
    );
    print(
      '║ TLV default dest: ${ServerConfig.gcsHostSystemIp}:${ServerConfig.udpStreamingPort} ║',
    );
    print('║ RSSI mode       : ${mode.padRight(22)}║');
    print('╚══════════════════════════════════════╝');
    print('');
    print('Default RSSI TLV matches §5 Table 2 (109 bytes) when --manual-sample.');
    print('Control: GET/POST http://0.0.0.0:${ServerConfig.radioHttpPort}/control/radio-state');
    print('  POST body: {"state":"normal"|"warning"|"critical"|"disconnected"}');
    print('');
  }
}
