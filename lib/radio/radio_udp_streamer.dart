import 'dart:async';
import 'dart:io';

import '../config/server_config.dart';
import 'radio_http_server.dart';
import 'radio_simulator.dart';
import 'radio_tlv_encoder.dart';

/// Streams Silvus TLV UDP reports to addresses configured via JSON-RPC.
class RadioUdpStreamer {
  RadioUdpStreamer({
    required RadioSimulator simulator,
    required RadioHttpServer httpServer,
    RadioTlvEncoder encoder = const RadioTlvEncoder(),
  })  : _simulator = simulator,
        _httpServer = httpServer,
        _encoder = encoder;

  final RadioSimulator _simulator;
  final RadioHttpServer _httpServer;
  final RadioTlvEncoder _encoder;

  RawDatagramSocket? _socket;
  Timer? _rssiTimer;
  Timer? _tempTimer;

  Future<void> start() async {
    _socket = await RawDatagramSocket.bind(InternetAddress.anyIPv4, 0);
    print('[RADIO] TLV UDP socket bound on ephemeral port');
  }

  void updateTimers() {
    _rssiTimer?.cancel();
    _tempTimer?.cancel();

    final rssiPeriod = Duration(
      milliseconds: _httpServer.rssiReportPeriodMs.clamp(
        ServerConfig.rssiReportPeriodMinMs,
        ServerConfig.rssiReportPeriodMaxMs,
      ),
    );
    final tempPeriod = Duration(
      seconds: _httpServer.tempReportPeriodSec.clamp(1, 2147483647),
    );

    _rssiTimer = Timer.periodic(rssiPeriod, (_) => _sendRssiIfEnabled());
    _tempTimer = Timer.periodic(tempPeriod, (_) => _sendTemperatureIfEnabled());
  }

  Future<void> stop() async {
    _rssiTimer?.cancel();
    _tempTimer?.cancel();
    _socket?.close();
    _socket = null;
  }

  void _sendRssiIfEnabled() {
    if (!_httpServer.rssiReportingEnabled || !_simulator.isStreaming) return;
    final sample = _nextSample();
    _sendReport(
      _encoder.encodeRssiReport(sample),
      host: _httpServer.rssiHost,
      port: _httpServer.rssiPort,
      kind: 'RSSI',
    );
  }

  void _sendTemperatureIfEnabled() {
    if (!_simulator.isStreaming) return;
    final sample = _nextSample();
    if (!_httpServer.shouldSendTemperature(sample)) return;
    _sendReport(
      _encoder.encodeTemperatureReport(sample),
      host: _httpServer.tempHost,
      port: _httpServer.tempPort,
      kind: 'TEMP',
    );
  }

  SimulatedRadioSample _nextSample() {
    final sample = _simulator.nextSample();
    _httpServer.lastSample = sample;
    return sample;
  }

  void _sendReport(
    List<int> bytes, {
    required String host,
    required int port,
    required String kind,
  }) {
    final socket = _socket;
    if (socket == null) return;

    final address = InternetAddress(host);
    final sent = socket.send(bytes, address, port);
    if (sent > 0) {
      print(
        '[RADIO] $kind TLV → $host:$port '
        '(${bytes.length} bytes, state=${_simulator.state.label})',
      );
    }
  }
}
