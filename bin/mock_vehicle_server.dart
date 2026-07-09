import 'dart:async';
import 'dart:io';
import 'dart:math' as math;
import 'dart:typed_data';

import 'package:mock_vehicle_server/config/server_config.dart';
import 'package:mock_vehicle_server/radio/radio_http_server.dart';
import 'package:mock_vehicle_server/radio/radio_simulator.dart';
import 'package:mock_vehicle_server/radio/radio_udp_streamer.dart';
import 'package:mavlink_dart/dialects/ugvcustom.dart';
import 'package:mavlink_dart/mavlink_frame.dart';
import 'package:mavlink_dart/mavlink_parser.dart';
import 'package:mavlink_dart/mavlink_signature.dart';

// ── CONFIG (mirrors scout-td0-GCS RadioConfig in app_constants.dart) ─────
const String gcsHost = ServerConfig.mavlinkHost;
const int gcsPort = ServerConfig.gcsPort;
const int serverPort = ServerConfig.vehicleUdpPort;

const int vehicleSysId = ServerConfig.vehicleSysId;
const int vehicleCompId = ServerConfig.vehicleCompId;
const int gcsSysId = ServerConfig.gcsSysId;
const int gcsCompId = ServerConfig.gcsCompId;

const int vehicleClockOffsetMs = ServerConfig.vehicleClockOffsetMs;
const int mavlinkLinkId = ServerConfig.mavlinkLinkId;

final Uint8List _mavlinkSecretKey = Uint8List.fromList([
  0x5f, 0xb2, 0x1c, 0x3a, 0x9e, 0xd4, 0x6f, 0x0b,
  0x2c, 0x7e, 0xa1, 0x4d, 0x83, 0x5b, 0xc9, 0x3f,
  0x0e, 0x6d, 0xb8, 0x94, 0x2a, 0xf1, 0xc5, 0x76,
  0x40, 0xed, 0x99, 0x13, 0xab, 0x5c, 0xe2, 0x04,
]);

// MAV_CMD values from ICD / ugvcustom.xml
const int cmdSetMode = 176;
const int cmdArmDisarm = 400;
const int cmdSetHome = 179;
const int cmdDriveMode = 31900;
const int cmdLightControl = 31901;
const int cmdCameraMarker = 31902;
const int cmdRemoteEmergency = 31904;

const int homeLat = 125234567; // 12.5234567°
const int homeLon = 801234567; // 80.1234567°
// ──────────────────────────────────────────────────────────────────────────

late RawDatagramSocket _socket;
final InternetAddress _gcsAddress = InternetAddress(gcsHost);
int _sequence = 0;
int _tickCount = 0;
late DateTime _bootTime;
late final MavlinkParser _parser;
late final MavlinkSignatureManager _signatureManager;

late final RadioSimulator _radioSimulator;
late final RadioHttpServer _radioHttpServer;
late final RadioUdpStreamer _radioUdpStreamer;

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

// Link health: 1=Healthy, 2=Degraded, 3=Faulty
const int linkHealthy = 1;
const int linkDegraded = 2;

// ── Mock UGV state (updated each tick / by commands) ─────────────────────
class _MockUgvState {
  int vcuOpState = vcuDrive;
  int chargerConnected = 0;
  int charging = 0;
  int towEngaged = 0;

  int lvBatterySoc = 78;
  int hvBatterySoc = 65;

  int autonomyMode = 1; // Mode A
  int holdState = 1; // 1=Disengaged, 2=Engaged
  int armMode = 2; // 1=Disarmed, 2=Armed, 3=Override
  int driveLimit = 2; // 1=Low, 2=Med, 3=High
  int driveMode = 1; // 1=Speed, 2=Torque, 3=Torque+limit

  int vehicleEStop = 1; // 1=Disengaged, 2=Engaged, 3=Disabled
  int remoteEStop = 1;
  int selectedCameraStream = 1; // Forward single
  int rangeMarkerOn = 0;

  // vcu_subsystem_status (ICD bytes 25-28)
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
  int lvChargerHealth = icdHealthy;
  int vcuHealth = icdHealthy;

  // comp_subsystem_status (ICD bytes 29-30)
  int uhfRadioState = icdHealthy;
  int lBandRadioState = 2; // ICD: 2=True/connected
  int ethernetSwitchState = 2; // ICD: 2=True
  int gnssState = icdHealthy;
  int imuState = icdHealthy;
  int lidar2dState = icdHealthy;
  int lidar3dState = icdHealthy;

  // UHF link (sensor_subsystem_health_2)
  int uhfConnection = linkConnected;
  int uhfLinkHealth = linkHealthy;

  // L-band link (sensor_subsystem_health_3 bit fields)
  int lBandConnection = linkConnected;
  int lBandLinkHealth = linkHealthy;
  int lBandSensorFaults = 0;

  // Motor / MC faults
  int motorFaults = 0;
  int mcFaults1 = 0;
  int pduFault = 0;
  int hvBatteryFaults = 0;
  int lvBatteryFaults = 0;

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
  int secondaryComputePower = pwrOn;
  int rgbdCameraPower = pwrOn;

  // VCU / compute interface health
  int vcuInterfaceHealth = 0;
  int compInterfaceHealth1 = 0;
  int secCompStatus = 0x003F; // interfaces active, secondary compute healthy
  int compInterfaceHealth2 = 0x01FF; // interfaces + GMSL 1-8 active

  bool homeSet = true;
  bool pathSaving = false;

  // Camera health in sensor_subsystem_health_4: 1=Healthy per 2-bit field
  int cameraHealthWord = 0x5555;

  // GNSS / attitude simulation
  int gpsAltMm = 920000; // 920 m MSL
  int gpsHeadingCd = 4500; // 45.00°
  int gpsSpeedCms = 0;
  double rollRad = 0.02;
  double pitchRad = -0.01;
  double yawRad = 0.78;
  double rollSpeedRad = 0.0;
}

final _state = _MockUgvState();

Future<void> main() async {
  _bootTime = DateTime.now();
  _signatureManager = MavlinkSignatureManager(
    MavlinkSignatureConfig(
      secretKey: _mavlinkSecretKey,
      linkId: mavlinkLinkId,
      acceptPolicy: SignatureAcceptPolicy.acceptUnsigned,
    ),
  );
  _parser = MavlinkParser(
    MavlinkDialectUgvcustom(),
    signatureManager: _signatureManager,
  );

  _radioSimulator = RadioSimulator();
  _radioHttpServer = RadioHttpServer(simulator: _radioSimulator);
  _radioUdpStreamer = RadioUdpStreamer(
    simulator: _radioSimulator,
    httpServer: _radioHttpServer,
  );
  _radioHttpServer.onConfigChanged = _radioUdpStreamer.updateTimers;

  _parser.stream.listen(_handleIncomingFrame);

  await _radioHttpServer.start();
  await _radioUdpStreamer.start();
  _radioUdpStreamer.updateTimers();

  _socket = await RawDatagramSocket.bind(InternetAddress.anyIPv4, serverPort);
  _socket.broadcastEnabled = true;

  print('╔══════════════════════════════════════╗');
  print('║    Mock Vehicle UDP Server           ║');
  print('╠══════════════════════════════════════╣');
  print('║ Bound on        : 0.0.0.0:$serverPort║');
  print('║ Sending to GCS  : $gcsHost:$gcsPort  ║');
  print('║ Vehicle sys/comp: $vehicleSysId / $vehicleCompId ║');
  print('║ Clock offset    : ${vehicleClockOffsetMs}ms║');
  print('╠══════════════════════════════════════╣');
  print('║ Silvus HTTP     : ${ServerConfig.gcsRadioIp}:${ServerConfig.radioHttpPort} ║');
  print('║ TLV default dest: ${ServerConfig.gcsHostSystemIp}:${ServerConfig.udpStreamingPort} ║');
  print('╚══════════════════════════════════════╝');

  _socket.listen((RawSocketEvent event) {
    if (event != RawSocketEvent.read) return;
    final dg = _socket.receive();
    if (dg == null) return;
    _parser.parse(dg.data);
  });

  Timer.periodic(const Duration(seconds: 1), (_) {
    _tickCount++;
    _radioSimulator.tick();
    _sendCompHeartbeat();
    _sendRadioStatus();
    _sendSystemTime();
  });

  Timer.periodic(const Duration(seconds: 1), (_) {
    _tickUgvSystemInfo();
    final ugv = _buildUgvSystemInfo();
    final sent = _send(ugv);
    print(
      '[UGV] sent → $sent bytes',
    );
  });

  Timer.periodic(const Duration(seconds: 1), (_) {
    _sendGpsRawInt();
    _sendAttitude();
  });

  print('\nServer running. Press Ctrl+C to stop.\n');
  await Future<void>.delayed(const Duration(days: 365));
}

void _handleIncomingFrame(MavlinkFrame frame) {
  final msg = frame.message;

  if (msg is Heartbeat) {
    print('[RX] GCS heartbeat — sysId=${frame.systemId} comp=${frame.componentId}');
    return;
  }

  if (msg is Timesync && msg.tc1 == 0) {
    _respondToTimesync(msg);
    return;
  }

  if (msg is CommandLong) {
    _handleCommandLong(msg, frame);
    return;
  }

  if (msg is ManualControl) {
    print('[RX] MANUAL_CONTROL x=${msg.x} y=${msg.y}');
    return;
  }

  print('[RX] Unhandled message: ${msg.runtimeType}');
}

void _respondToTimesync(Timesync request) {
  final vehTime = _vehicleTimeUtc();
  final tc1 = vehTime.microsecondsSinceEpoch;

  final response = Timesync(
    ts1: request.ts1,
    tc1: tc1,
    targetSystem: gcsSysId,
    targetComponent: gcsCompId,
  );

  final sent = _send(response);
  print('[TS] Response sent tc1=$tc1 us → $sent bytes');
}

void _handleCommandLong(CommandLong cmd, MavlinkFrame frame) {
  if (cmd.targetSystem != vehicleSysId || cmd.targetComponent != vehicleCompId) {
    print(
      '[RX] COMMAND_LONG ignored — target ${cmd.targetSystem}/${cmd.targetComponent}',
    );
    return;
  }

  final s = _state;
  final p1 = cmd.param1.round();
  final p2 = cmd.param2.round();
  final p3 = cmd.param3.round();

  switch (cmd.command) {
    case cmdSetMode:
      if (p1 == 1) {
        if (p2 >= 1 && p2 <= 5) s.autonomyMode = p2;
        if (p3 >= 1 && p3 <= 3) s.driveLimit = p3;
        print('[CMD] SET_MODE mode=$p2 limit=$p3');
      }
    case cmdArmDisarm:
      if (p1 >= 1 && p1 <= 3) {
        s.armMode = p1;
        print('[CMD] ARM_DISARM mode=$p1 force=$p2');
      }
    case cmdDriveMode:
      if (p1 >= 1 && p1 <= 3) {
        s.driveMode = p1;
        print('[CMD] DRIVE_MODE mode=$p1');
      }
    case cmdLightControl:
      if (p1 == 0 || p1 == 1) s.headlights = p1 == 1 ? pwrOn : pwrOff;
      if (p2 == 0 || p2 == 1) s.fogLights = p2 == 1 ? pwrOn : pwrOff;
      if (p3 == 0 || p3 == 1) s.aftLights = p3 == 1 ? pwrOn : pwrOff;
      print('[CMD] LIGHT_CONTROL head=$p1 fog=$p2 aft=$p3');
    case cmdCameraMarker:
      if (p1 >= 1 && p1 <= 6) s.selectedCameraStream = p1;
      if (p2 == 0 || p2 == 1) s.rangeMarkerOn = p2;
      print('[CMD] CAMERA_MARKER stream=$p1 marker=$p2');
    case cmdRemoteEmergency:
      if (p1 >= 1 && p1 <= 3) {
        s.remoteEStop = p1;
        print('[CMD] REMOTE_EMERGENCY state=$p1');
      }
    case cmdSetHome:
      if (p1 == 1) {
        s.homeSet = true;
        print('[CMD] SET_HOME current position');
      }
    default:
      print(
        '[RX] COMMAND_LONG cmd=${cmd.command} from '
        '${frame.systemId}/${frame.componentId}',
      );
  }
}

void _sendCompHeartbeat() {
  // COMP_HEARTBEAT to GCS (ICD 5.1.6.2.1): base_mode=0, custom_mode=0
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
}

void _sendSystemTime() {
  final vehTime = _vehicleTimeUtc();
  final bootMs = DateTime.now().difference(_bootTime).inMilliseconds;

  final st = SystemTime(
    timeUnixUsec: BigInt.from(vehTime.microsecondsSinceEpoch),
    timeBootMs: bootMs,
  );

  final sent = _send(st);
  print(
    '[ST] one-time boot=${bootMs}ms unix=${vehTime.microsecondsSinceEpoch} → $sent bytes',
  );
}

void _sendGpsRawInt() {
  final vehTime = _vehicleTimeUtc();
  final s = _state;

  final gps = GpsRawInt(
    timeUsec: BigInt.from(vehTime.microsecondsSinceEpoch),
    lat: homeLat,
    lon: homeLon,
    alt: s.gpsAltMm,
    eph: 120,
    epv: 65535,
    vel: s.gpsSpeedCms,
    cog: s.gpsHeadingCd,
    fixType: 2, // 3D fix
    satellitesVisible: 14,
  );

  _send(gps);
}

void _sendAttitude() {
  final bootMs = DateTime.now().difference(_bootTime).inMilliseconds;
  final s = _state;

  final att = Attitude(
    timeBootMs: bootMs,
    roll: s.rollRad,
    pitch: s.pitchRad,
    yaw: s.yawRad,
    rollspeed: s.rollSpeedRad,
    pitchspeed: 0,
    yawspeed: 0,
  );

  _send(att);
}

void _sendRadioStatus() {
  if (!_radioSimulator.isStreaming) return;

  final sample = _radioSimulator.nextSample();
  _radioHttpServer.lastSample = sample;

  final radio = RadioStatus(
    rxerrors: 0,
    fixed: 0,
    rssi: sample.mavlinkRssi,
    remrssi: sample.mavlinkRemRssi,
    txbuf: 100,
    noise: sample.mavlinkNoise,
    remnoise: sample.mavlinkRemNoise,
  );

  final sent = _send(radio);
  print(
    '[RADIO_STATUS] rssi=${sample.antenna1Dbm}dBm '
    'rem=${sample.antenna2Dbm}dBm noise=${sample.noiseDbm}dBm → $sent bytes',
  );
}

void _applyRadioStateToUgv() {
  final s = _state;
  switch (_radioSimulator.state) {
    case RadioSimulationState.normal:
      s.lBandRadioState = 2;
      s.lBandConnection = linkConnected;
      s.lBandLinkHealth = linkHealthy;
      s.lBandSensorFaults = 0;
      s.lBandPower = pwrOn;
    case RadioSimulationState.warning:
      s.lBandRadioState = 2;
      s.lBandConnection = linkConnected;
      s.lBandLinkHealth = linkDegraded;
      s.lBandSensorFaults = (1 << 2) | (1 << 6); // RSSI + local RSSI health
      s.lBandPower = pwrOn;
    case RadioSimulationState.critical:
      s.lBandRadioState = icdFaulty;
      s.lBandConnection = linkConnected;
      s.lBandLinkHealth = 3; // Faulty
      s.lBandSensorFaults = 0x1F; // multiple L-band fault bits
      s.lBandPower = pwrOn;
    case RadioSimulationState.disconnected:
      s.lBandRadioState = icdUnknown;
      s.lBandConnection = 2; // Disconnected
      s.lBandLinkHealth = icdUnknown;
      s.lBandSensorFaults = 1 << 4; // heartbeat fault
      s.lBandPower = pwrOff;
  }
}

DateTime _vehicleTimeUtc() {
  return DateTime.now().toUtc().add(
    const Duration(milliseconds: vehicleClockOffsetMs),
  );
}

// ── Build UGV_SYSTEM_INFO per ICD v1.3 + ugvcustom.xml ────────────────────
UgvSystemInfo _buildUgvSystemInfo() {
  final s = _state;

  final vcuStatus = _packFields(
    [s.vcuOpState, s.chargerConnected, s.charging, s.towEngaged],
    [2, 2, 2, 2],
  );

  final batterySoc = (s.hvBatterySoc << 8) | (s.lvBatterySoc & 0xFF);

  final compMode1 = _packFields(
    [s.autonomyMode, s.holdState, s.armMode, s.driveLimit, s.driveMode],
    [4, 2, 2, 4, 4],
  );

  final compMode2 = _packFields(
    [
      s.vehicleEStop,
      s.remoteEStop,
      s.selectedCameraStream,
      s.rangeMarkerOn,
    ],
    [2, 2, 3, 1],
  );

  final vcuSubsystemStatus = _packFields(
    [
      s.aftMcHealth,
      s.fwdMcHealth,
      s.hvBatteryHealth,
      s.lvBatteryHealth,
      s.lvPduHealth,
      s.dcDcHealth,
      s.hvPduHealth,
      s.fwdLeftMotorHealth,
      s.aftLeftMotorHealth,
      s.fwdRightMotorHealth,
      s.aftRightMotorHealth,
      s.mainComputeHealth,
      s.lvChargerHealth,
      s.vcuHealth,
    ],
    [2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2],
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

  final sensorSubsystemHealth2 = _packFields(
    [s.uhfConnection, s.uhfLinkHealth],
    [2, 2],
  );

  final sensorSubsystemHealth3 = _packLBandHealth(
    faultBits: s.lBandSensorFaults,
    connection: s.lBandConnection,
    linkHealth: s.lBandLinkHealth,
  );

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
      s.gnssPower,
      s.imuPower,
      s.lidar2dPower,
      s.lidar3dPower,
      s.vcuPower,
      s.mainComputePower,
      s.secondaryComputePower,
      s.rgbdCameraPower,
    ],
    [2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2],
  );

  final vcuPowerSubsystemState2 = _packFields(
    [s.headlights, s.aftLights, s.fogLights],
    [2, 2, 2],
  );

  final homeLocation = _packFields(
    [s.homeSet ? 1 : 0, s.pathSaving ? 1 : 0],
    [1, 1],
  );

  return UgvSystemInfo(
    vcuStatus: vcuStatus,
    batterySoc: batterySoc,
    compMode1: compMode1,
    compMode2: compMode2,
    sensorSubsystemHealth1: 0,
    sensorSubsystemHealth2: sensorSubsystemHealth2,
    sensorSubsystemHealth3: sensorSubsystemHealth3,
    sensorSubsystemHealth4: s.cameraHealthWord,
    vcuSubsystemStatus: vcuSubsystemStatus,
    compSubsystemStatus: compSubsystemStatus,
    vcuSubsystemPowerState1: vcuSubsystemPowerState1,
    vcuPowerSubsystemState2: vcuPowerSubsystemState2,
    motorFaults: s.motorFaults,
    validityMotorFaults: 0,
    mcFaults1: s.mcFaults1,
    mcFaults2: 0,
    contactorFault: 0,
    pduFault: s.pduFault,
    powerSubsystemFaults1: s.hvBatteryFaults,
    powerSubsystemFaults2: s.lvBatteryFaults,
    vcuInterfaceHealth: s.vcuInterfaceHealth,
    secCompStatus: s.secCompStatus,
    compInterfaceHealth1: s.compInterfaceHealth1,
    compInterfaceHealth2: s.compInterfaceHealth2,
    homeLocation: homeLocation,
    lat: s.homeSet ? homeLat : 0,
    lon: s.homeSet ? homeLon : 0,
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
      signatureManager: _signatureManager,
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
  s.lvChargerHealth = icdHealthy;
  s.vcuHealth = icdHealthy;

  s.uhfRadioState = icdHealthy;
  s.lBandRadioState = 2;
  s.ethernetSwitchState = 2;
  s.gnssState = icdHealthy;
  s.imuState = icdHealthy;
  s.lidar2dState = icdHealthy;
  s.lidar3dState = icdHealthy;

  s.uhfConnection = linkConnected;
  s.uhfLinkHealth = linkHealthy;

  s.lBandConnection = linkConnected;
  s.lBandLinkHealth = linkHealthy;
  s.lBandSensorFaults = 0;

  s.motorFaults = 0;
  s.mcFaults1 = 0;
  s.pduFault = 0;
  s.hvBatteryFaults = 0;
  s.lvBatteryFaults = 0;
  s.vcuInterfaceHealth = 0;
  s.compInterfaceHealth1 = 0;
  s.secCompStatus = 0x003F;
  s.compInterfaceHealth2 = 0x01FF;
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
  s.gpsSpeedCms = 0;
  s.rollSpeedRad = 0.0;

  // Slowly drain batteries
  s.lvBatterySoc = (s.lvBatterySoc > 20) ? s.lvBatterySoc - 1 : 85;
  s.hvBatterySoc = (s.hvBatterySoc > 15) ? s.hvBatterySoc - 1 : 90;

  // Gentle attitude drift for IMU telemetry
  s.yawRad = (s.yawRad + 0.002) % (2 * math.pi);
  s.rollRad = 0.02 * math.sin(_tickCount * 0.1);
  s.pitchRad = -0.01 * math.cos(_tickCount * 0.1);

  // ── Mode / arm cycling ───────────────────────────────────────────────
  if (phase < 10) {
    s.autonomyMode = 1;
    s.holdState = 1;
    s.armMode = 2;
    s.driveLimit = 1;
    s.driveMode = 1;
    s.vcuOpState = vcuDrive;
  } else if (phase < 20) {
    s.autonomyMode = 2;
    s.holdState = 2;
    s.armMode = 2;
    s.driveLimit = 2;
    s.driveMode = 2;
    s.vcuOpState = vcuKeyOn;
    s.chargerConnected = 1;
    s.charging = 1;
    s.gpsSpeedCms = 150;
  } else {
    s.autonomyMode = 1;
    s.holdState = 1;
    s.armMode = 1;
    s.driveLimit = 3;
    s.driveMode = 1;
    s.vcuOpState = vcuIdle;
    s.towEngaged = 1;
  }

  // ── Fault scenarios ────────────────────────────────────────────────────
  if (phase >= 5 && phase < 10) {
    s.fwdLeftMotorHealth = icdFaulty;
    s.motorFaults = _packMotorFaults(
      aftPort: 0,
      aftStbd: 0,
      fwdPort: 0x20,
      fwdStbd: 0,
    );
  } else if (phase >= 10 && phase < 15) {
    s.uhfRadioState = icdFaulty;
    s.uhfConnection = linkConnected;
    s.uhfLinkHealth = linkDegraded;
  } else if (phase >= 15 && phase < 20) {
    s.dcDcHealth = icdFaulty;
  } else if (phase >= 20 && phase < 25) {
    s.hvBatteryHealth = icdFaulty;
    s.hvBatteryFaults = 1 << 3;
    s.mcFaults1 = 1 << 3;
    s.fwdMcHealth = icdFaulty;
  } else if (phase >= 25) {
    s.mainComputeHealth = icdFaulty;
    s.compInterfaceHealth1 = 1;
    s.secCompStatus = 0x0240; // secondary compute faulty
    s.pduFault = 0x0F;
    s.vehicleEStop = 2;
    s.headlights = pwrOff;
    s.fogLights = pwrOn;
    s.cameraHealthWord = 0x55A5; // port camera faulty
  }

  _applyRadioStateToUgv();
}

// ── Bit-field helpers ─────────────────────────────────────────────────────

/// Pack L-band sensor_subsystem_health_3 per ugvcustom.xml bytes 19-21.
int _packLBandHealth({
  required int faultBits,
  required int connection,
  required int linkHealth,
}) {
  var word = faultBits & 0x1FFF;
  word |= (connection & 0x3) << 13;
  word |= (linkHealth & 0x3) << 16;
  return word;
}

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
