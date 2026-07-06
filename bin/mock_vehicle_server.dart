import 'dart:async';
import 'dart:io';

import 'package:mavlink_dart/dialects/ugvcustom.dart';
import 'package:mavlink_dart/mavlink_frame.dart';
import 'package:mavlink_dart/mavlink_parser.dart';

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

// ── ICD encodings (COMP_UGV_STATUS / UGV_SYSTEM_INFO v1.3) ───────────────

// Subsystem / link health: 0=Unknown, 1=Healthy, 2=Faulty, 3=Reserved
const int icdUnknown = 0;
const int icdHealthy = 1;
const int icdFaulty = 2;

// Power state: 0=Unknown, 1=ON, 2=OFF
const int pwrOn = 1;
const int pwrOff = 2;

// VCU operational state: 1=Idle, 2=Key On, 3=Drive
const int vcuIdle = 1;
const int vcuKeyOn = 2;
const int vcuDrive = 3;

// Link connection: 1=Connected, 2=Disconnected
const int linkConnected = 1;
const int linkDisconnected = 2;

// Link health: 1=Healthy, 2=Degraded, 3=Faulty
const int linkHealthy = 1;
const int linkDegraded = 2;
const int linkFaulty = 3;

// ── Mock UGV state (updated each tick) ────────────────────────────────────
class _MockUgvState {
  int vcuOpState = vcuDrive;
  int chargerConnected = 0;
  int charging = 0;
  int towEngaged = 0;

  int lvBatterySoc = 78;
  int hvBatterySoc = 65;

  int autonomyMode = 1; // Mode A
  int holdState = 1; // 1=Disengaged, 2=Engaged
  int armMode = 2; // 1=Disarmed, 2=Armed
  int driveLimit = 2; // 1=Low, 2=Med, 3=High
  int driveMode = 1; // 1=Speed, 2=Torque

  int vehicleEStop = 1; // 1=Disengaged, 2=Engaged
  int remoteEStop = 1;

  // vcu_subsystem_status health (ICD bytes 25-28)
  int aftMcHealth = icdHealthy;
  int fwdMcHealth = icdHealthy;
  int hvBatteryHealth = icdHealthy;
  int lvBatteryHealth = icdHealthy;
  int lvPduHealth = icdHealthy;
  int dcDcHealth = icdHealthy;
  int hvPduHealth = icdHealthy;
  int fwdLeftMotorHealth = icdHealthy;
  int aftLeftMotorHealth = icdHealthy;
  int fwdRightMotorHealth = icdHealthy;
  int aftRightMotorHealth = icdHealthy;
  int mainComputeHealth = icdHealthy;
  int secondaryComputeHealth = icdHealthy;
  int vcuHealth = icdHealthy;

  // comp_subsystem_status (ICD bytes 29-30)
  int uhfRadioState = icdHealthy;
  int lBandRadioState = 2; // ICD: 2=True/connected
  int ethernetSwitchState = icdHealthy;
  int gnssState = icdHealthy;
  int imuState = icdHealthy;
  int lidar2dState = icdHealthy;
  int lidar3dState = icdHealthy;

  // UHF link (sensor_subsystem_health_2)
  int uhfConnection = linkConnected;
  int uhfLinkHealth = linkHealthy;

  // Motor / MC faults
  int motorFaults = 0; // uint32: 4 x uint8 motor fault bytes
  int mcFaults1 = 0; // uint16: aft | fwd<<8
  int pduFault = 0; // uint8 channel faults in low byte
  int hvBatteryFaults = 0; // power_subsystem_faults_1
  int lvBatteryFaults = 0; // power_subsystem_faults_2 low byte

  // Lights: 1=ON, 2=OFF
  int headlights = pwrOn;
  int aftLights = pwrOn;
  int fogLights = pwrOff;

  // Power rails (1=ON, 2=OFF)
  int fwdMcPower = pwrOn;
  int aftMcPower = pwrOn;
  int dcDcPower = pwrOn;
  int hvPduPower = pwrOn;
  int lvPduPower = pwrOn;
  int uhfPower = pwrOn;
  int lBandPower = pwrOn;
  int ethernetPower = pwrOn;
  int gnssPower = pwrOn;
  int imuPower = pwrOn;
  int lidar2dPower = pwrOn;
  int lidar3dPower = pwrOn;
  int vcuPower = pwrOn;
  int mainComputePower = pwrOn;

  // VCU / compute interface faults
  int vcuInterfaceHealth = 0;
  int compInterfaceHealth1 = 0; // jetson heartbeat etc.
  int secCompStatus = 0;

  // Camera health in sensor_subsystem_health_4: 1=Healthy, 2=Faulty
  int cameraHealthWord = 0x5555; // all healthy (01 in each 2-bit field)
}

final _state = _MockUgvState();

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
  print('║ Message         : UGV_SYSTEM_INFO    ║');
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
      type: 18,
      autopilot: 8,
      baseMode: 0,
      customMode: 0,
      systemStatus: 4,
      mavlinkVersion: 3,
    );

    final sent = _send(hb);
    print('[HB] tick=$_tickCount → $sent bytes');
  });

  Timer.periodic(const Duration(seconds: 1), (_) {
    final now = DateTime.now().toUtc();
    final vehTime = now.add(const Duration(milliseconds: vehicleClockOffsetMs));
    final bootMs = now.difference(_bootTime).inMilliseconds;

    final st = SystemTime(
      timeUnixUsec: BigInt.from(vehTime.microsecondsSinceEpoch),
      timeBootMs: bootMs,
    );

    final sent = _send(st);
    print(
      '[ST] boot=${bootMs}ms unix=${vehTime.microsecondsSinceEpoch} → $sent bytes',
    );
  });

  Timer.periodic(const Duration(seconds: 1), (_) {
    _tickUgvSystemInfo();
    final ugv = _buildUgvSystemInfo();

    final sent = _send(ugv);
    print(
      '[UGV] vcu=${ugv.vcuStatus} soc=LV${ugv.batterySoc & 0xFF}/'
      'HV${ugv.batteryHvSoc} mode=${ugv.compModel} → $sent bytes',
    );
  });

  print('\nServer running. Press Ctrl+C to stop.\n');
  await Future<void>.delayed(const Duration(days: 365));
}

// ── Build UGV_SYSTEM_INFO per ICD + mavlink_dart layout ───────────────────
UgvSystemInfo _buildUgvSystemInfo() {
  final s = _state;

  final vcuStatus = _packFields(
    [s.vcuOpState, s.chargerConnected, s.charging, s.towEngaged],
    [2, 2, 2, 2],
  );

  final compModel = _packFields(
    [s.autonomyMode, s.holdState, s.armMode, s.driveLimit, s.driveMode],
    [4, 2, 2, 4, 4],
  );

  final compMode2 = _packFields(
    [
      s.vehicleEStop,
      s.remoteEStop,
      1, // forward camera stream
      0, // range marker off
    ],
    [2, 2, 3, 1],
  );

  final vcuSubsystemStatus = _packFields(
    [s.aftMcHealth, s.fwdMcHealth, s.hvBatteryHealth, s.lvBatteryHealth],
    [2, 2, 2, 2],
  );

  final computeNodeHealth = _packFields(
    [
      s.lvPduHealth,
      s.dcDcHealth,
      s.hvPduHealth,
      s.fwdLeftMotorHealth,
      s.aftLeftMotorHealth,
      s.fwdRightMotorHealth,
      s.aftRightMotorHealth,
      s.mainComputeHealth,
    ],
    [2, 2, 2, 2, 2, 2, 2, 2],
  );

  final jetsonInterfaceHealth = _packFields(
    [s.secondaryComputeHealth, s.vcuHealth],
    [2, 2],
  );

  final compSubsystemStatus = _packFields(
    [
      s.uhfRadioState,
      s.lBandRadioState,
      s.ethernetSwitchState,
      s.gnssState,
      s.imuState,
      s.lidar2dState,
      s.lidar3dState,
    ],
    [2, 2, 2, 2, 2, 2, 2],
  );

  final sensorSubsystemHealth1 = 0; // no UHF faults
  final sensorSubsystemHealth2 = _packFields(
    [s.uhfConnection, s.uhfLinkHealth],
    [2, 2],
  );
  final sensorSubsystemHealth3 = 0; // no L-band radio faults (byte 19)
  final sensorSubsystemHealth4 = s.cameraHealthWord;

  final vcuSubsystemPowerState1 = _packFields(
    [
      s.fwdMcPower,
      s.aftMcPower,
      s.dcDcPower,
      s.hvPduPower,
      s.lvPduPower,
      s.uhfPower,
      s.lBandPower,
      s.ethernetPower,
    ],
    [2, 2, 2, 2, 2, 2, 2, 2],
  );

  final powerHighByte = _packFields(
    [
      s.gnssPower,
      s.imuPower,
      s.lidar2dPower,
      s.lidar3dPower,
      s.vcuPower,
      s.mainComputePower,
      pwrOn, // secondary compute
      pwrOn, // RGBD camera
    ],
    [2, 2, 2, 2, 2, 2, 2, 2],
  );

  final lightsByte = _packFields(
    [s.headlights, s.aftLights, s.fogLights],
    [2, 2, 2],
  );

  final vcuPowerSubsystemState2 = (powerHighByte << 8) | lightsByte;

  const homeLat = 125234567; // 12.5234567°
  const homeLon = 801234567; // 80.1234567°

  return UgvSystemInfo(
    motorFaults: s.motorFaults,
    compInterfaceHealth2: 0,
    lat: homeLat,
    lon: homeLon,
    batterySoc: s.lvBatterySoc & 0xFF,
    compModel: compModel,
    sensorSubsystemHealth1: sensorSubsystemHealth1,
    sensorSubsystemHealth4: sensorSubsystemHealth4,
    compSubsystemStatus: compSubsystemStatus,
    vcuSubsystemPowerState1: vcuSubsystemPowerState1,
    vcuPowerSubsystemState2: vcuPowerSubsystemState2,
    validityMotorFaults: 0,
    mcFaults1: s.mcFaults1,
    mcFaults2: 0,
    contactorFault: 0,
    pduFault: s.pduFault,
    powerSubsystemFaults1: s.hvBatteryFaults,
    powerSubsystemFaults2: s.lvBatteryFaults,
    vcuInterfaceHealth: s.vcuInterfaceHealth,
    compInterfaceHealth1: s.compInterfaceHealth1,
    homeLocation: 1,
    // home set
    pduFaultExtended: 0,
    powerSubsystemFaultsExtended: 0,
    vcuInterfaceHealthExtended: 0,
    vcuBusFaults: 0,
    vcuInternalFaults: 0,
    vcuHardwareSafety: 0,
    auxCanInterfaces: 0,
    computeNodeHealth: computeNodeHealth,
    jetsonInterfaceHealth: jetsonInterfaceHealth,
    visionGmslHealth: 0,
    vcuStatus: vcuStatus,
    batteryHvSoc: s.hvBatterySoc,
    compMode2: compMode2,
    sensorSubsystemHealth2: sensorSubsystemHealth2,
    sensorSubsystemHealth3: sensorSubsystemHealth3,
    vcuSubsystemStatus: vcuSubsystemStatus,
    secCompStatus: s.secCompStatus,
    latPlaceholder: 0,
    longPlaceholder: 0,
    jetsonHeartbeat: 0,
    homeInitialization: 1,
  );
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

// ── Mock state evolution (cycles through scenarios every 30 s) ────────────
void _tickUgvSystemInfo() {
  final phase = _tickCount % 30;
  final s = _state;

  // Reset to healthy baseline
  s.aftMcHealth = icdHealthy;
  s.fwdMcHealth = icdHealthy;
  s.hvBatteryHealth = icdHealthy;
  s.lvBatteryHealth = icdHealthy;
  s.lvPduHealth = icdHealthy;
  s.dcDcHealth = icdHealthy;
  s.hvPduHealth = icdHealthy;
  s.fwdLeftMotorHealth = icdHealthy;
  s.aftLeftMotorHealth = icdHealthy;
  s.fwdRightMotorHealth = icdHealthy;
  s.aftRightMotorHealth = icdHealthy;
  s.mainComputeHealth = icdHealthy;
  s.secondaryComputeHealth = icdHealthy;
  s.vcuHealth = icdHealthy;

  s.uhfRadioState = icdHealthy;
  s.lBandRadioState = 2;
  s.ethernetSwitchState = icdHealthy;
  s.gnssState = icdHealthy;
  s.imuState = icdHealthy;
  s.lidar2dState = icdHealthy;
  s.lidar3dState = icdHealthy;

  s.uhfConnection = linkConnected;
  s.uhfLinkHealth = linkHealthy;

  s.motorFaults = 0;
  s.mcFaults1 = 0;
  s.pduFault = 0;
  s.hvBatteryFaults = 0;
  s.lvBatteryFaults = 0;
  s.vcuInterfaceHealth = 0;
  s.compInterfaceHealth1 = 0;
  s.secCompStatus = 0;
  s.cameraHealthWord = 0x5555;

  s.vcuOpState = vcuDrive;
  s.chargerConnected = 0;
  s.charging = 0;
  s.towEngaged = 0;
  s.vehicleEStop = 1;
  s.remoteEStop = 1;
  s.headlights = pwrOn;
  s.aftLights = pwrOn;
  s.fogLights = pwrOff;

  // Slowly drain batteries
  s.lvBatterySoc = (s.lvBatterySoc > 20) ? s.lvBatterySoc - 1 : 85;
  s.hvBatterySoc = (s.hvBatterySoc > 15) ? s.hvBatterySoc - 1 : 90;

  // ── Mode / arm cycling ───────────────────────────────────────────────
  if (phase < 10) {
    s.autonomyMode = 1; // Mode A
    s.holdState = 1;
    s.armMode = 2; // Armed
    s.driveLimit = 1;
    s.driveMode = 1; // Speed
    s.vcuOpState = vcuDrive;
  } else if (phase < 20) {
    s.autonomyMode = 2; // Mode B
    s.holdState = 2; // Hold engaged
    s.armMode = 2;
    s.driveLimit = 2;
    s.driveMode = 2; // Torque
    s.vcuOpState = vcuKeyOn;
    s.chargerConnected = 1;
    s.charging = 1;
  } else {
    s.autonomyMode = 1;
    s.holdState = 1;
    s.armMode = 1; // Disarmed
    s.driveLimit = 3;
    s.driveMode = 1;
    s.vcuOpState = vcuIdle;
    s.towEngaged = 1;
  }

  // ── Fault scenarios ────────────────────────────────────────────────────
  if (phase >= 5 && phase < 10) {
    // Front left motor over-temperature
    s.fwdLeftMotorHealth = icdFaulty;
    s.motorFaults = _packMotorFaults(
      aftPort: 0,
      aftStbd: 0,
      fwdPort: 0x20, // over-temperature bit 5
      fwdStbd: 0,
    );
  } else if (phase >= 10 && phase < 15) {
    // UHF degraded link
    s.uhfRadioState = icdFaulty;
    s.uhfConnection = linkConnected;
    s.uhfLinkHealth = linkDegraded;
  } else if (phase >= 15 && phase < 20) {
    // DC-DC unhealthy
    s.dcDcHealth = icdFaulty;
  } else if (phase >= 20 && phase < 25) {
    // HV battery + rear MC undervoltage
    s.hvBatteryHealth = icdFaulty;
    s.hvBatteryFaults = 1 << 3; // pack undervoltage
    s.mcFaults1 = 1 << 3; // aft MC undervoltage
    s.fwdMcHealth = icdFaulty;
  } else if (phase >= 25) {
    // Compute offline + PDU channel faults + e-stop
    s.mainComputeHealth = icdFaulty;
    s.compInterfaceHealth1 = 1; // jetson heartbeat fault
    s.pduFault = 0x0F; // channels 1-4 fault
    s.vehicleEStop = 2; // engaged
    s.headlights = pwrOff;
    s.fogLights = pwrOn;
    s.cameraHealthWord = 0x5549; // port camera faulty
  }
}

// ── Bit-field helpers ─────────────────────────────────────────────────────

/// Pack a list of integer field values at given bit widths (LSB first).
int _packFields(List<int> values, List<int> widths) {
  var result = 0;
  var shift = 0;
  for (var i = 0; i < values.length; i++) {
    final mask = (1 << widths[i]) - 1;
    result |= (values[i] & mask) << shift;
    shift += widths[i];
  }
  return result;
}

/// Pack four motor fault bytes into motor_faults uint32 (ICD bytes 36-39).
int _packMotorFaults({
  required int aftPort,
  required int aftStbd,
  required int fwdPort,
  required int fwdStbd,
}) {
  return (aftPort & 0xFF) |
      ((aftStbd & 0xFF) << 8) |
      ((fwdPort & 0xFF) << 16) |
      ((fwdStbd & 0xFF) << 24);
}
