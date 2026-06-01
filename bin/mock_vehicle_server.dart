import 'dart:async';
import 'dart:io';
import 'dart:typed_data';

import 'package:mavlink_module/dialects/ugvcustom.dart';
import 'package:mavlink_module/mavlink_frame.dart';
import 'package:mavlink_module/mavlink_parser.dart';

// ── CONFIG ────────────────────────────────────────────────────────────────
const String gcsHost = '127.0.0.1';
const int gcsPort = 7500;
const int serverPort = 7000;

const int vehicleSysId = 1;
const int vehicleCompId = 190;
const int gcsSysId = 255;
const int gcsCompId = 190;

const int vehicleClockOffsetMs = -500;
// ──────────────────────────────────────────────────────────────────────────

late RawDatagramSocket _socket;
final InternetAddress _gcsAddress = InternetAddress(gcsHost);
int _sequence = 0;
int _tickCount = 0;
late DateTime _bootTime;
late final MavlinkParser _parser;

// ── Mock UGV state ────────────────────────────────────────────────────────
int _batterySoc = 92;

// Health encoding from XML:
// 0 = reserved, 1 = no communication, 2 = healthy, 3 = unhealthy
const int _hsReserved = 0;
const int _hsNoComm = 1;
const int _hsHealthy = 2;
const int _hsUnhealthy = 3;

int _vcuHealth = _hsHealthy;
int _hvBatteryHealth = _hsHealthy;
int _lvBatteryHealth = _hsHealthy;
int _lvPduHealth = _hsHealthy;
int _dcDc48To12Health = _hsHealthy;
int _dcDc12To5Health = _hsHealthy;

int _frontLeftMotorHealth = _hsHealthy;
int _rearLeftMotorHealth = _hsHealthy;
int _frontRightMotorHealth = _hsHealthy;
int _rearRightMotorHealth = _hsHealthy;

int _leftMcHealth = _hsHealthy;
int _rightMcHealth = _hsHealthy;

int _uhfHealth = _hsHealthy;
int _lBandHealth = _hsHealthy;
int _computeHealth = _hsHealthy;

// Fault bitmasks
int _rearLeftMotorFaults = 0;
int _rearRightMotorFaults = 0;
int _frontLeftMotorFaults = 0;
int _frontRightMotorFaults = 0;

int _rearMcFaults = 0;
int _frontMcFaults = 0;

// Metrics
int _rearMcVoltage = 482; // 48.2V if your generated class allows >255, else adjust
int _frontMcVoltage = 481;
int _rearMcTemperature = 36;
int _frontMcTemperature = 35;

// Packed status bytes
int _lightStatus = 0;
int _pduChannelStatus = 0xFF;

// Modes
int _mainMode = 1;
int _subMode = 0;
int _speedMode = 1;
int _driveMode = 0;
int _armMode = mavBoolTrue;

int _intendedMainMode = 1;
int _intendedSubMode = 0;
int _intendedSpeedMode = 1;
int _intendedDriveMode = 0;
int _intendedArmMode = mavBoolTrue;
int _modeChangeReason = 0;

Future<void> main() async {
  _bootTime = DateTime.now();
  _parser = MavlinkParser(MavlinkDialectUgvcustom());

  _parser.stream.listen((frame) {
    final msg = frame.message;

    if (msg is Heartbeat) {
      print('[RX] GCS heartbeat — sysId=${frame.systemId}');
      return;
    }

    if (msg is Timesync && msg.tc1 == 0) {
      Future.delayed(const Duration(milliseconds: 20), () {
        final now = DateTime.now().toUtc();
        final vehTime = now.add(
          const Duration(milliseconds: vehicleClockOffsetMs),
        );
        final nowNs = vehTime.microsecondsSinceEpoch * 1000;

        final response = Timesync(
          ts1: msg.ts1,
          tc1: nowNs,
          targetSystem: gcsSysId,
          targetComponent: gcsCompId,
        );

        final sent = _send(response);
        print('[TS] Response sent tc1=$nowNs → $sent bytes');
      });
      return;
    }

    print('[RX] Unhandled message: ${msg.runtimeType}');
  });

  _socket = await RawDatagramSocket.bind(InternetAddress.anyIPv4, serverPort);
  _socket.broadcastEnabled = true;

  print('╔══════════════════════════════════════╗');
  print('║    Mock Vehicle UDP Server           ║');
  print('╠══════════════════════════════════════╣');
  print('║ Bound on        : 0.0.0.0:$serverPort║');
  print('║ Sending to GCS  : $gcsHost:$gcsPort  ║');
  print('║ Clock offset    : ${vehicleClockOffsetMs}ms║');
  print('╚══════════════════════════════════════╝');

  _socket.listen((RawSocketEvent event) {
    if (event != RawSocketEvent.read) return;
    final dg = _socket.receive();
    if (dg == null) return;
    _parser.parse(dg.data);
  });

  Timer.periodic(const Duration(seconds: 1), (_) {
    _tickCount++;

    final hb = Heartbeat(
      type: mavTypeOnboardController,
      autopilot: mavAutopilotInvalid,
      baseMode: 0,
      customMode: 0,
      systemStatus: mavStateActive,
      mavlinkVersion: 3,
    );

    final sent = _send(hb);
    print('[HB] tick=$_tickCount → $sent bytes');
  });

  Timer.periodic(const Duration(seconds: 1), (_) {
    final now = DateTime.now().toUtc();
    final vehTime = now.add(const Duration(milliseconds: vehicleClockOffsetMs));
    final bootMs = now
        .difference(_bootTime)
        .inMilliseconds;

    final st = SystemTime(
      timeUnixUsec: vehTime.microsecondsSinceEpoch,
      timeBootMs: bootMs,
    );

    final sent = _send(st);
    print(
      '[ST] boot=${bootMs}ms unix=${vehTime
          .microsecondsSinceEpoch} → $sent bytes',
    );
  });

  Timer.periodic(const Duration(seconds: 1), (_) {
    _tickUgvSystemInfo();

    final now = DateTime.now();
    final ugv = UgvSystemInfo(
      ts1Hour: now.hour,
      ts1Minute: now.minute,
      ts1Second: now.second,

      subsystemHealth1: _pack4Health(
        _leftMcHealth,
        _rightMcHealth,
        _hvBatteryHealth,
        _lvBatteryHealth,
      ),
      subsystemHealth2: _pack4Health(
        _lvPduHealth,
        _dcDc48To12Health,
        _dcDc12To5Health,
        _vcuHealth,
      ),
      subsystemHealth3: _pack4Health(
        _frontLeftMotorHealth,
        _rearLeftMotorHealth,
        _frontRightMotorHealth,
        _rearRightMotorHealth,
      ),
      subsystemHealth4: _pack4Health(
        _uhfHealth,
        _lBandHealth,
        _computeHealth,
        _hsReserved,
      ),

      ts2Hour: now.hour,
      ts2Minute: now.minute,
      ts2Second: now.second,

      batterySoc: _batterySoc,

      mainMode: _mainMode,
      subMode: _subMode,
      speedMode: _speedMode,
      driveMode: _driveMode,
      armMode: _armMode,

      intendedMainMode: _intendedMainMode,
      intendedSubMode: _intendedSubMode,
      intendedSpeedMode: _intendedSpeedMode,
      intendedDriveMode: _intendedDriveMode,
      intendedArmMode: _intendedArmMode,
      modeChangeReason: _modeChangeReason,

      ts3Hour: now.hour,
      ts3Minute: now.minute,
      ts3Second: now.second,

      rearLeftMotorFaults: _rearLeftMotorFaults,
      rearRightMotorFaults: _rearRightMotorFaults,
      frontLeftMotorFaults: _frontLeftMotorFaults,
      frontRightMotorFaults: _frontRightMotorFaults,

      rearMcFaults: _rearMcFaults,
      frontMcFaults: _frontMcFaults,

      rearMcVoltage: _rearMcVoltage,
      frontMcVoltage: _frontMcVoltage,
      rearMcTemperature: _rearMcTemperature,
      frontMcTemperature: _frontMcTemperature,

      lightStatus: _lightStatus,
      pduChannelStatus: _pduChannelStatus,
    );

    final sent = _send(ugv);
    print(
      '[UGV] health1=${ugv.subsystemHealth1} health2=${ugv.subsystemHealth2} '
          'health3=${ugv.subsystemHealth3} health4=${ugv.subsystemHealth4} '
          'soc=${ugv.batterySoc} → $sent bytes',
    );
  });

  print('\nServer running. Press Ctrl+C to stop.\n');
  await Future<void>.delayed(const Duration(days: 365));
}

// ── Serialize + send ─────────────────────────────────────────────────────
int _send(dynamic message) {
  try {
    final frame = MavlinkFrame.v2(
      _sequence++ & 0xFF,
      vehicleSysId,
      vehicleCompId,
      message,
    );
    final bytes = frame.serialize();
    return _socket.send(bytes.buffer.asUint8List(), _gcsAddress, gcsPort);
  } catch (e) {
    print('[ERR] Send failed: $e');
    return -1;
  }
}

// ── Mock state evolution ─────────────────────────────────────────────────
void _tickUgvSystemInfo() {
  final phase = _tickCount % 30;

  // Default everything healthy/no faults
  _vcuHealth = _hsHealthy;
  _hvBatteryHealth = _hsHealthy;
  _lvBatteryHealth = _hsHealthy;
  _lvPduHealth = _hsHealthy;
  _dcDc48To12Health = _hsHealthy;
  _dcDc12To5Health = _hsHealthy;

  _frontLeftMotorHealth = _hsHealthy;
  _rearLeftMotorHealth = _hsHealthy;
  _frontRightMotorHealth = _hsHealthy;
  _rearRightMotorHealth = _hsHealthy;

  _leftMcHealth = _hsHealthy;
  _rightMcHealth = _hsHealthy;

  _uhfHealth = _hsHealthy;
  _lBandHealth = _hsHealthy;
  _computeHealth = _hsHealthy;

  _rearLeftMotorFaults = 0;
  _rearRightMotorFaults = 0;
  _frontLeftMotorFaults = 0;
  _frontRightMotorFaults = 0;
  _rearMcFaults = 0;
  _frontMcFaults = 0;

  _rearMcVoltage = 482;
  _frontMcVoltage = 481;
  _rearMcTemperature = 36;
  _frontMcTemperature = 35;

  _lightStatus = _buildLightStatus(
    headLight: phase >= 18,
    frontFog: phase >= 22,
    rearLight: true,
  );

  _pduChannelStatus = 0xFF;

  _batterySoc = (_batterySoc > 15) ? _batterySoc - 1 : 95;

  if (phase >= 5 && phase < 10) {
    _frontLeftMotorHealth = _hsUnhealthy;
    _frontLeftMotorFaults = 0x20; // over temp
    _frontMcTemperature = 72;
  } else if (phase >= 10 && phase < 15) {
    _uhfHealth = _hsNoComm;
  } else if (phase >= 15 && phase < 20) {
    _dcDc48To12Health = _hsUnhealthy;
  } else if (phase >= 20 && phase < 25) {
    _hvBatteryHealth = _hsUnhealthy;
    _rearMcFaults = 0x08; // under voltage
    _rearMcVoltage = 438;
  } else if (phase >= 25) {
    _computeHealth = _hsNoComm;
    _pduChannelStatus = 0xb11101111;
  }

  if (phase < 10) {
    _mainMode = 1;
    _subMode = 0;
    _speedMode = 1;
    _driveMode = 0;
    _armMode = mavBoolTrue;
    _modeChangeReason = 0;
  } else if (phase < 20) {
    _mainMode = 2;
    _subMode = 1;
    _speedMode = 2;
    _driveMode = 1;
    _armMode = mavBoolTrue;
    _modeChangeReason = 0;
  } else {
    _mainMode = 2;
    _subMode = 1;
    _speedMode = 0;
    _driveMode = 2;
    _armMode = mavBoolFalse;
    _modeChangeReason = 3;
  }

  _intendedMainMode = _mainMode;
  _intendedSubMode = _subMode;
  _intendedSpeedMode = _speedMode;
  _intendedDriveMode = _driveMode;
  _intendedArmMode = _armMode;
}

// ── Helpers ──────────────────────────────────────────────────────────────
int _pack4Health(int a, int b, int c, int d) {
  return (a & 0x03) |
  ((b & 0x03) << 2) |
  ((c & 0x03) << 4) |
  ((d & 0x03) << 6);
}

int _buildLightStatus({
  required bool headLight,
  required bool frontFog,
  required bool rearLight,
}) {
  var value = 0;
  if (headLight) value |= 1 << 0;
  if (frontFog) value |= 1 << 1;
  if (rearLight) value |= 1 << 2;
  return value;
}