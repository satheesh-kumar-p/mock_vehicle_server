import 'dart:async';
import 'dart:convert';
import 'dart:io';

import '../config/server_config.dart';
import 'radio_simulator.dart';

/// Silvus StreamScape JSON-RPC mock (§§3.10–3.21, 5 & 6) plus a control API.
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

  String rssiHost = ServerConfig.gcsHostSystemIp;
  int rssiPort = ServerConfig.udpStreamingPort;
  String tempHost = ServerConfig.gcsHostSystemIp;
  int tempPort = ServerConfig.udpStreamingPort;

  int rssiReportPeriodMs = ServerConfig.udpReportTimeMs;
  int tempReportPeriodSec = ServerConfig.udpTemperatureReportPeriodSec;
  int tempMinThresholdC = ServerConfig.tempReportingMinThresholdC;
  int tempMaxThresholdC = ServerConfig.tempReportingMaxThresholdC;

  /// 0 = disabled, 1 = heating/overheating only, 2 = periodic (§3.20).
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
        'useManualSample': _simulator.useManualSample,
        'keepaliveNoTraffic': _simulator.keepaliveNoTraffic,
        'rssiReportingEnabled': rssiReportingEnabled,
        'rssiDestination': '$rssiHost:$rssiPort',
        'tempDestination': '$tempHost:$tempPort',
        'tempReportingMode': tempReportingMode,
        'availableStates': RadioSimulationState.values
            .map((state) => state.label)
            .toList(),
      });
      return;
    }

    if (request.method == 'POST') {
      final body = await utf8.decoder.bind(request).join();
      final payload = body.isEmpty ? <String, dynamic>{} : jsonDecode(body);

      if (payload.containsKey('keepaliveNoTraffic')) {
        _simulator.keepaliveNoTraffic =
            payload['keepaliveNoTraffic'] == true ||
                payload['keepaliveNoTraffic']?.toString() == '1';
      }
      if (payload.containsKey('useManualSample')) {
        _simulator.useManualSample =
            payload['useManualSample'] == true ||
                payload['useManualSample']?.toString() == '1';
      }

      if (payload.containsKey('state')) {
        final parsed = RadioSimulationStateLabel.tryParse(
          payload['state']?.toString() ?? '',
        );
        if (parsed == null) {
          request.response.statusCode = HttpStatus.badRequest;
          _writeJson(request.response, {
            'error':
                'Invalid state. Use: normal, warning, critical, disconnected',
          });
          return;
        }
        _simulator.setState(parsed);
      }

      _writeJson(request.response, {
        'state': _simulator.state.label,
        'streaming': _simulator.isStreaming,
        'useManualSample': _simulator.useManualSample,
        'keepaliveNoTraffic': _simulator.keepaliveNoTraffic,
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
    final params =
        (payload['params'] as List?)?.map((e) => e.toString()).toList() ?? [];
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
        if (params.isEmpty) {
          return [rssiReportingEnabled ? '1' : '0'];
        }
        rssiReportingEnabled = params.first == '1';
        print('[HTTP] rssi_report_enable → $rssiReportingEnabled');
        onConfigChanged?.call();
        return const [];
      case 'rssi_report_address':
        if (params.isEmpty) {
          return [rssiHost, '$rssiPort'];
        }
        if (params.length >= 2) {
          rssiHost = params[0];
          rssiPort = int.tryParse(params[1]) ?? rssiPort;
          print('[HTTP] rssi_report_address → $rssiHost:$rssiPort');
          onConfigChanged?.call();
        }
        return const [];
      case 'rssi_report_period':
        if (params.isEmpty) {
          return ['$rssiReportPeriodMs'];
        }
        final parsed = int.tryParse(params.first);
        if (parsed != null) {
          rssiReportPeriodMs = parsed.clamp(
            ServerConfig.rssiReportPeriodMinMs,
            ServerConfig.rssiReportPeriodMaxMs,
          );
          print('[HTTP] rssi_report_period → ${rssiReportPeriodMs}ms');
          onConfigChanged?.call();
        }
        return const [];
      case 'temp_reporting_address':
        if (params.isEmpty) {
          return [tempHost, '$tempPort'];
        }
        if (params.length >= 2) {
          tempHost = params[0];
          tempPort = int.tryParse(params[1]) ?? tempPort;
          print('[HTTP] temp_reporting_address → $tempHost:$tempPort');
          onConfigChanged?.call();
        }
        return const [];
      case 'temp_reporting_max_threshold':
        if (params.isEmpty) {
          return ['$tempMaxThresholdC'];
        }
        if (params.isNotEmpty) {
          tempMaxThresholdC =
              int.tryParse(params.first) ?? tempMaxThresholdC;
          _simulator.setTempMaxThreshold(tempMaxThresholdC);
        }
        return const [];
      case 'temp_reporting_min_threshold':
        if (params.isEmpty) {
          return ['$tempMinThresholdC'];
        }
        if (params.isNotEmpty) {
          tempMinThresholdC =
              int.tryParse(params.first) ?? tempMinThresholdC;
        }
        return const [];
      case 'temp_reporting_period':
        if (params.isEmpty) {
          return ['$tempReportPeriodSec'];
        }
        if (params.isNotEmpty) {
          tempReportPeriodSec =
              int.tryParse(params.first) ?? tempReportPeriodSec;
          onConfigChanged?.call();
        }
        return const [];
      case 'temp_reporting_mode':
        if (params.isEmpty) {
          return ['$tempReportingMode'];
        }
        if (params.isNotEmpty) {
          final mode = int.tryParse(params.first);
          if (mode != null && mode >= 0 && mode <= 2) {
            tempReportingMode = mode;
            print('[HTTP] temp_reporting_mode → $tempReportingMode');
            onConfigChanged?.call();
          }
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

  /// Whether a temperature report should be sent for [sample] under §3.20.
  bool shouldSendTemperature(SimulatedRadioSample sample) {
    switch (tempReportingMode) {
      case 0:
        return false;
      case 1:
        final t = sample.temperatureCelsius;
        // Heating: between min and max thresholds; overheating: above max.
        return t >= tempMinThresholdC;
      case 2:
        return true;
      default:
        return false;
    }
  }

  void _writeJson(HttpResponse response, Map<String, dynamic> body) {
    response.headers.contentType = ContentType.json;
    response.write(jsonEncode(body));
    response.close();
  }
}
