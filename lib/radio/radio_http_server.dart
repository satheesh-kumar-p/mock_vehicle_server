import 'dart:async';
import 'dart:convert';
import 'dart:io';

import '../config/server_config.dart';
import 'radio_simulator.dart';

/// Silvus StreamScape JSON-RPC mock (Sections 5 & 6) plus a control API for
/// manual UI verification.
class RadioHttpServer {
  RadioHttpServer({
    required RadioSimulator simulator,
    String host = ServerConfig.radioHttpBindHost,
    int port = ServerConfig.radioHttpPort,
  })  : _simulator = simulator,
        _host = host,
        _port = port;

  final RadioSimulator _simulator;
  final String _host;
  final int _port;

  HttpServer? _server;

  bool rssiReportingEnabled = false;
  String tlvHost = ServerConfig.gcsHostSystemIp;
  int tlvPort = ServerConfig.udpStreamingPort;
  int rssiReportPeriodMs = ServerConfig.udpReportTimeMs;
  int tempReportPeriodSec = ServerConfig.udpTemperatureReportPeriodSec;
  int tempMinThresholdC = ServerConfig.tempReportingMinThresholdC;
  int tempMaxThresholdC = ServerConfig.tempReportingMaxThresholdC;
  int tempReportingMode = 2;

  SimulatedRadioSample? lastSample;
  void Function()? onConfigChanged;

  Future<void> start() async {
    _server = await HttpServer.bind(_host, _port);
    print(
      '[HTTP] Silvus API on http://${ServerConfig.gcsRadioIp}:$_port/streamscape_api',
    );
    print(
      '[HTTP] Control API  on http://$_host:$_port/control/radio-state',
    );

    _server!.listen(_handleRequest);
  }

  Future<void> stop() async {
    await _server?.close(force: true);
    _server = null;
  }

  Future<void> _handleRequest(HttpRequest request) async {
    try {
      final path = request.uri.path;

      if (path == '/control/radio-state') {
        await _handleControl(request);
        return;
      }

      if (path == '/streamscape_api' && request.method == 'POST') {
        await _handleJsonRpc(request);
        return;
      }

      request.response
        ..statusCode = HttpStatus.notFound
        ..write('Not found')
        ..close();
    } catch (e) {
      request.response
        ..statusCode = HttpStatus.internalServerError
        ..write('Internal error: $e')
        ..close();
    }
  }

  Future<void> _handleControl(HttpRequest request) async {
    if (request.method == 'GET') {
      _writeJson(request.response, {
        'state': _simulator.state.label,
        'streaming': _simulator.isStreaming,
        'rssiReportingEnabled': rssiReportingEnabled,
        'tlvDestination': '$tlvHost:$tlvPort',
        'availableStates': RadioSimulationState.values
            .map((state) => state.label)
            .toList(),
      });
      return;
    }

    if (request.method == 'POST') {
      final body = await utf8.decoder.bind(request).join();
      final payload = body.isEmpty ? <String, dynamic>{} : jsonDecode(body);

      final parsed = RadioSimulationStateLabel.tryParse(
        payload['state']?.toString() ?? '',
      );
      if (parsed == null) {
        request.response.statusCode = HttpStatus.badRequest;
        _writeJson(request.response, {
          'error': 'Invalid state. Use: normal, warning, critical, disconnected',
        });
        return;
      }

      _simulator.setState(parsed);
      _writeJson(request.response, {
        'state': _simulator.state.label,
        'streaming': _simulator.isStreaming,
      });
      return;
    }

    request.response.statusCode = HttpStatus.methodNotAllowed;
    await request.response.close();
  }

  Future<void> _handleJsonRpc(HttpRequest request) async {
    final body = await utf8.decoder.bind(request).join();
    final payload = jsonDecode(body) as Map<String, dynamic>;
    final method = payload['method']?.toString() ?? '';
    final params = (payload['params'] as List?)?.map((e) => e.toString()).toList() ?? [];
    final id = payload['id'];

    final result = _dispatchJsonRpc(method, params);

    _writeJson(request.response, {
      'jsonrpc': '2.0',
      'result': result,
      'id': id,
    });
  }

  List<String> _dispatchJsonRpc(String method, List<String> params) {
    switch (method) {
      case 'rssi_report_enable':
        rssiReportingEnabled = params.isNotEmpty && params.first == '1';
        print('[HTTP] rssi_report_enable → $rssiReportingEnabled');
        onConfigChanged?.call();
        return const [];
      case 'rssi_report_address':
        if (params.length >= 2) {
          tlvHost = params[0];
          tlvPort = int.tryParse(params[1]) ?? tlvPort;
          print('[HTTP] rssi_report_address → $tlvHost:$tlvPort');
          onConfigChanged?.call();
        }
        return const [];
      case 'rssi_report_period':
        if (params.isNotEmpty) {
          rssiReportPeriodMs = int.tryParse(params.first) ?? rssiReportPeriodMs;
          print('[HTTP] rssi_report_period → ${rssiReportPeriodMs}ms');
          onConfigChanged?.call();
        }
        return const [];
      case 'temp_reporting_address':
        if (params.length >= 2) {
          tlvHost = params[0];
          tlvPort = int.tryParse(params[1]) ?? tlvPort;
          print('[HTTP] temp_reporting_address → $tlvHost:$tlvPort');
          onConfigChanged?.call();
        }
        return const [];
      case 'temp_reporting_max_threshold':
        if (params.isNotEmpty) {
          tempMaxThresholdC = int.tryParse(params.first) ?? tempMaxThresholdC;
        }
        return const [];
      case 'temp_reporting_min_threshold':
        if (params.isNotEmpty) {
          tempMinThresholdC = int.tryParse(params.first) ?? tempMinThresholdC;
        }
        return const [];
      case 'temp_reporting_period':
        if (params.isNotEmpty) {
          tempReportPeriodSec =
              int.tryParse(params.first) ?? tempReportPeriodSec;
          onConfigChanged?.call();
        }
        return const [];
      case 'temp_reporting_mode':
        if (params.isNotEmpty) {
          tempReportingMode = int.tryParse(params.first) ?? tempReportingMode;
        }
        return const [];
      case 'read_current_temperature':
        final sample = lastSample ?? _simulator.nextSample();
        return ['${sample.temperatureCelsius}'];
      default:
        print('[HTTP] Unknown JSON-RPC method: $method');
        return const [];
    }
  }

  void _writeJson(HttpResponse response, Map<String, dynamic> body) {
    response.headers.contentType = ContentType.json;
    response.write(jsonEncode(body));
    response.close();
  }
}
