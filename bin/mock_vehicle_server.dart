import 'dart:async';
import 'dart:io';
import 'dart:math' as math;
import 'dart:typed_data';

import 'package:mock_vehicle_server/config/server_config.dart';
import 'package:mavlink_dart/dialects/ugvcustom.dart';
import 'package:mavlink_dart/mavlink_frame.dart';
import 'package:mavlink_dart/mavlink_parser.dart';
import 'package:mavlink_dart/mavlink_signature.dart';

// ── CONFIG (mirrors scout-td0-GCS RadioConfig in app_constants.dart) ─────
// Silvus radio mock runs separately: dart run bin/mock_radio_server.dart
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

// Bangalore map center — fixed GPS orbit center (independent of SET_HOME)
const int orbitLat = 130828500; // 13.08285°
const int orbitLon = 776234500; // 77.62345°

// Continuous circle for marker-animation testing
const double gpsCircleRadiusM = 500.0;
const double gpsCircleSpeedMs = 1.5; // ~1.5 m/s
const double metersPerDegLat = 111320.0;
// ──────────────────────────────────────────────────────────────────────────

late RawDatagramSocket _socket;
final InternetAddress _gcsAddress = InternetAddress(gcsHost);
int _sequence = 0;
int _tickCount = 0;
late DateTime _bootTime;
late final MavlinkParser _parser;
late final MavlinkSignatureManager _signatureManager;
Process? _gstreamerProcess;

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

  bool homeSet = false;
  bool pathSaving = false;

  // Home reported in UGV_SYSTEM_INFO — latched from GPS_RAW_INT on SET_HOME
  int homeLatE7 = 0;
  int homeLonE7 = 0;

  // Camera health in sensor_subsystem_health_4: 1=Healthy per 2-bit field
  int cameraHealthWord = 0x5555;

  // GNSS / attitude simulation (orbits fixed [orbitLat]/[orbitLon], not home)
  int gpsLatE7 = orbitLat;
  int gpsLonE7 = orbitLon;
  double gpsCircleTheta = 0.0; // rad; angle on circle around orbit center
  int gpsAltMm = 920000; // 920 m MSL
  int gpsHeadingCd = 4500; // 45.00°
  int gpsSpeedCms = 150; // 1.50 m/s while circling
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

  _parser.stream.listen(_handleIncomingFrame);

  _socket = await RawDatagramSocket.bind(InternetAddress.anyIPv4, serverPort);
  _socket.broadcastEnabled = true;

  print('╔══════════════════════════════════════╗');
  print('║    Mock Vehicle UDP Server           ║');
  print('╠══════════════════════════════════════╣');
  print('║ Bound on        : 0.0.0.0:$serverPort║');
  print('║ Sending to GCS  : $gcsHost:$gcsPort  ║');
  print('║ Vehicle sys/comp: $vehicleSysId / $vehicleCompId ║');
  print('║ Clock offset    : ${vehicleClockOffsetMs}ms║');
  print(
    '║ Video RTP       : ${ServerConfig.videoStreamHost}:'
    '${ServerConfig.videoStreamPort}║',
  );
  print('╚══════════════════════════════════════╝');
  print('Silvus radio: dart run bin/mock_radio_server.dart');

  await _startGstreamerVideo();

  _socket.listen((RawSocketEvent event) {
    if (event != RawSocketEvent.read) return;
    final dg = _socket.receive();
    if (dg == null) return;
    _parser.parse(dg.data);
  });

  Timer.periodic(const Duration(seconds: 1), (_) {
    _tickCount++;
    _sendCompHeartbeat();
    _sendSystemTime();
  });

  Timer.periodic(const Duration(seconds: 1), (_) {
    _tickUgvSystemInfo();
    final ugv = _buildUgvSystemInfo();
    final sent = _send(ugv);
    final s = _state;
    print(
      '[UGV] sent → $sent bytes'
      '${s.homeSet ? ' home=${s.homeLatE7},${s.homeLonE7}' : ' home=unset'}'
      ' gps=${s.gpsLatE7},${s.gpsLonE7}',
    );
  });

  Timer.periodic(const Duration(seconds: 1), (_) {
    _tickGpsMotion();
    _sendGpsRawInt();
    _sendAttitude();
  });

  print('\nServer running. Press Ctrl+C to stop.\n');

  // Keep process alive until SIGINT/SIGTERM, then tear down GStreamer.
  final stop = Completer<void>();
  void requestStop(ProcessSignal signal) {
    if (stop.isCompleted) return;
    print('\n[$signal] Shutting down…');
    stop.complete();
  }

  ProcessSignal.sigint.watch().listen(requestStop);
  if (!Platform.isWindows) {
    ProcessSignal.sigterm.watch().listen(requestStop);
  }

  await stop.future;
  await _stopGstreamerVideo();
}

Future<void> _startGstreamerVideo() async {
  final host = ServerConfig.videoStreamHost;
  final port = ServerConfig.videoStreamPort;
  final args = <String>[
    '-v',
    'videotestsrc',
    'is-live=true',
    'pattern=ball',
    '!',
    'video/x-raw,width=1280,height=720,framerate=30/1',
    '!',
    'videoconvert',
    '!',
    'x264enc',
    'tune=zerolatency',
    'speed-preset=ultrafast',
    'key-int-max=30',
    '!',
    'rtph264pay',
    'pt=96',
    'config-interval=1',
    '!',
    'udpsink',
    'host=$host',
    'port=$port',
  ];

  try {
    final process = await Process.start('gst-launch-1.0', args);
    _gstreamerProcess = process;
    print('[GST] H.264 RTP test pattern → udp://$host:$port (pid=${process.pid})');

    process.stdout
        .transform(const SystemEncoding().decoder)
        .listen((line) => stdout.write('[GST] $line'));
    process.stderr
        .transform(const SystemEncoding().decoder)
        .listen((line) => stderr.write('[GST] $line'));
    process.exitCode.then((code) {
      if (identical(_gstreamerProcess, process)) {
        _gstreamerProcess = null;
      }
      print('[GST] exited with code $code');
    });
  } on ProcessException catch (e) {
    print(
      '[GST] Failed to start gst-launch-1.0 ($e). '
      'Install GStreamer or start the pipeline manually.',
    );
  }
}

Future<void> _stopGstreamerVideo() async {
  final process = _gstreamerProcess;
  if (process == null) return;
  _gstreamerProcess = null;
  print('[GST] Stopping pid=${process.pid}');
  process.kill(ProcessSignal.sigint);
  try {
    await process.exitCode.timeout(const Duration(seconds: 2));
  } on TimeoutException {
    process.kill(ProcessSignal.sigkill);
  }
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
      // ICD 5.1.6.2.12: param1=1 → latch current GPS_RAW_INT lat/lon as home
      if (p1 == 1) {
        s.homeSet = true;
        s.homeLatE7 = s.gpsLatE7;
        s.homeLonE7 = s.gpsLonE7;
        print(
          '[CMD] SET_HOME latched GPS_RAW_INT → '
          'lat=${s.homeLatE7} lon=${s.homeLonE7} '
          '(${s.homeLatE7 / 1e7}, ${s.homeLonE7 / 1e7})',
        );
        // Push UGV_SYSTEM_INFO immediately so GCS sees the latch
        // before the next GPS motion tick.
        final ugv = _buildUgvSystemInfo();
        final sent = _send(ugv);
        print(
          '[UGV] SET_HOME status → home lat=${ugv.lat} lon=${ugv.lon} '
          '($sent bytes)',
        );
      } else {
        print('[CMD] SET_HOME ignored — param1=$p1 (need 1)');
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

/// Advance circular GPS position + COG at 1 Hz around fixed [orbitLat]/[orbitLon].
/// Home (SET_HOME) is independent — latching home must not move the GPS orbit.
void _tickGpsMotion() {
  final s = _state;
  final omega = gpsCircleSpeedMs / gpsCircleRadiusM; // rad/s at 1 Hz ticks
  s.gpsCircleTheta = (s.gpsCircleTheta + omega) % (2 * math.pi);

  final orbitLatDeg = orbitLat / 1e7;
  final orbitLonDeg = orbitLon / 1e7;
  final metersPerDegLon =
      metersPerDegLat * math.cos(orbitLatDeg * math.pi / 180);

  // Offset from center: north = r·cos(θ), east = r·sin(θ)
  final northM = gpsCircleRadiusM * math.cos(s.gpsCircleTheta);
  final eastM = gpsCircleRadiusM * math.sin(s.gpsCircleTheta);

  final latDeg = orbitLatDeg + northM / metersPerDegLat;
  final lonDeg = orbitLonDeg + eastM / metersPerDegLon;

  s.gpsLatE7 = (latDeg * 1e7).round();
  s.gpsLonE7 = (lonDeg * 1e7).round();

  // Tangent COG for CCW motion (north/east velocity ∝ -sin(θ), cos(θ))
  final northVel = -math.sin(s.gpsCircleTheta);
  final eastVel = math.cos(s.gpsCircleTheta);
  var cogDeg = math.atan2(eastVel, northVel) * 180 / math.pi;
  if (cogDeg < 0) cogDeg += 360;

  s.gpsHeadingCd = (cogDeg * 100).round() % 36000;
  s.gpsSpeedCms = (gpsCircleSpeedMs * 100).round(); // cm/s
  s.yawRad = cogDeg * math.pi / 180;
}

void _sendGpsRawInt() {
  final vehTime = _vehicleTimeUtc();
  final s = _state;

  final gps = GpsRawInt(
    timeUsec: BigInt.from(vehTime.microsecondsSinceEpoch),
    lat: s.gpsLatE7,
    lon: s.gpsLonE7,
    alt: s.gpsAltMm,
    eph: 120,
    epv: 65535,
    vel: s.gpsSpeedCms,
    cog: s.gpsHeadingCd,
    fixType: 3, // 3D fix
    satellitesVisible: 14,
  );

  final sent = _send(gps);
  print(
    '[GPS] lat=${s.gpsLatE7 / 1e7} lon=${s.gpsLonE7 / 1e7} '
    'cog=${s.gpsHeadingCd / 100}° vel=${s.gpsSpeedCms / 100} m/s → $sent bytes',
  );
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
    lat: s.homeSet ? s.homeLatE7 : 0,
    lon: s.homeSet ? s.homeLonE7 : 0,
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
  // gpsSpeedCms / lat-lon are owned by _tickGpsMotion (continuous circle)
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
