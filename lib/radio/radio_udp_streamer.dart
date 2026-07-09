import 'dart:async';
import 'dart:io';

import 'radio_http_server.dart';
import 'radio_simulator.dart';
import 'radio_tlv_encoder.dart';

/// Streams Silvus TLV UDP reports to the address configured via JSON-RPC.
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
      milliseconds: _httpServer.rssiReportPeriodMs.clamp(100, 60000),
    );
    final tempPeriod = Duration(
      seconds: _httpServer.tempReportPeriodSec.clamp(1, 60),
    );

    _rssiTimer = Timer.periodic(rssiPeriod, (_) => _sendRssiIfEnabled());
    _tempTimer = Timer.periodic(tempPeriod, (_) => _sendTemperature());
  }

  Future<void> stop() async {
    _rssiTimer?.cancel();
    _tempTimer?.cancel();
    _socket?.close();
    _socket = null;
  }

  void _sendRssiIfEnabled() {
    if (!_httpServer.rssiReportingEnabled || !_simulator.isStreaming) return;
    _sendReport(_encoder.encodeRssiReport(_nextSample()));
  }

  void _sendTemperature() {
    if (!_simulator.isStreaming) return;
    _sendReport(_encoder.encodeTemperatureReport(_nextSample()));
  }

  SimulatedRadioSample _nextSample() {
    final sample = _simulator.nextSample();
    _httpServer.lastSample = sample;
    return sample;
  }

  void _sendReport(List<int> bytes) {
    final socket = _socket;
    if (socket == null) return;

    final address = InternetAddress(_httpServer.tlvHost);
    final sent = socket.send(bytes, address, _httpServer.tlvPort);
    if (sent > 0) {
      print(
        '[RADIO] TLV → ${_httpServer.tlvHost}:${_httpServer.tlvPort} '
        '(${bytes.length} bytes, state=${_simulator.state.label})',
      );
    }
  }
}
