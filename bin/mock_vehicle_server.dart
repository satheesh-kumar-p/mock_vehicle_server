import 'dart:async';
import 'dart:convert';
import 'dart:io';
import 'dart:math' as math;
import 'dart:typed_data';

import 'package:mock_vehicle_server/config/server_config.dart';
import 'package:scout_mavlink_dart/scout_mavlink_dart.dart';

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

// MAV_CMD values from ICD v1.4 / ugvcustom.xml
const int cmdSetMode = 176;
const int cmdArmDisarm = 400;
const int cmdSetHome = 179;
const int cmdRth = 20;
const int cmdDriveMode = 31900;
const int cmdLightControl = 31901;
const int cmdCameraMarker = 31902;
const int cmdOverrideSafety = 31903;
const int cmdRemoteEmergency = 31904;

// Bangalore map center — fixed GPS path center (independent of SET_HOME)
const int orbitLat = 130828500; // 13.08285°
const int orbitLon = 776234500; // 77.62345°

// Slow square path — small enough to stay on one map screen for SET_HOME testing
const double gpsSquareHalfSideM = 25.0; // 50 m × 50 m square
const double gpsSquareSpeedMs = 0.25; // ~0.25 m/s (~13 min per lap)
const double gpsTickSeconds = 0.1; // ICD 5.1.7.2.3 / .4: GPS + ATT at 10 Hz
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
final List<Timer> _timers = [];
var _shuttingDown = false;

/// High-rate messages (GPS/ATT at 10 Hz, MANUAL_CONTROL while driving) log at most 1 Hz.
const Duration _highRateLogInterval = Duration(seconds: 1);
final Map<Type, DateTime> _lastHighRateLogAt = {};

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

/// Polaris Connect Flow health outcomes for GCS Battery/Motor/Sensor POST.
enum HealthScenario {
  /// All battery / motor / sensor fields healthy (Figma Success_Health_Check).
  success,

  /// Motor group faulty only (Figma Error_Health_Check partial).
  partial,

  /// Battery + motor + sensor all faulty (Figma Error_Health_Check complete).
  failure,

  /// Legacy 30 s rotating fault / mode phases.
  cycle,
}

HealthScenario _healthScenario = HealthScenario.success;

HealthScenario? _parseHealthScenario(String raw) {
  switch (raw.trim().toLowerCase()) {
    case 'success':
      return HealthScenario.success;
    case 'partial':
      return HealthScenario.partial;
    case 'failure':
    case 'fail':
    case 'complete':
      return HealthScenario.failure;
    case 'cycle':
      return HealthScenario.cycle;
    default:
      return null;
  }
}

void _printUsage() {
  stdout.writeln('''
Usage: dart run bin/mock_vehicle_server.dart [options]

Options:
  --state=success|partial|failure|cycle
      Lock UGV_SYSTEM_INFO health to a Polaris Connect Flow scenario.
      Default: success

  -h, --help
      Show this help.

Scenarios (GCS Battery / Motor / Sensor POST rows):
  success   all pass  → "Connection Successful! …"
  partial   motor fail only → "Error Detected…"
  failure   all fail  → "Error Detected…"
  cycle     rotate faults every 30 s (legacy)
''');
}

HealthScenario? _parseArgs(List<String> args) {
  var scenario = HealthScenario.success;

  for (final arg in args) {
    if (arg == '--help' || arg == '-h') {
      _printUsage();
      return null;
    }

    if (arg.startsWith('--state=')) {
      final parsed = _parseHealthScenario(arg.substring('--state='.length));
      if (parsed == null) {
        stderr.writeln(
          'Unknown --state value. Use success|partial|failure|cycle.',
        );
        _printUsage();
        return null;
      }
      scenario = parsed;
      continue;
    }

    stderr.writeln('Unknown argument: $arg');
    _printUsage();
    return null;
  }

  return scenario;
}

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

  // GNSS / attitude simulation (square path around [orbitLat]/[orbitLon], not home)
  int gpsLatE7 = orbitLat;
  int gpsLonE7 = orbitLon;
  double gpsPathDistanceM = 0.0; // meters traveled along square perimeter
  int gpsAltMm = 920000; // 920 m MSL
  int gpsHeadingCd = 9000; // 90.00° (east, first square side)
  int gpsSpeedCms = 25; // 0.25 m/s while traversing square
  double rollRad = 0.02;
  double pitchRad = -0.01;
  double yawRad = math.pi / 2;
  double rollSpeedRad = 0.0;
}

final _state = _MockUgvState();

Future<void> main(List<String> args) async {
  final scenario = _parseArgs(args);
  if (scenario == null) {
    exit(64);
  }
  _healthScenario = scenario;

  _bootTime = DateTime.now();
  _signatureManager = MavlinkSignatureManager(
    MavlinkSignatureConfig(
      secretKey: _mavlinkSecretKey,
      linkId: mavlinkLinkId,
      // Match GCS acceptAll so signed COMMAND_LONG (SET_HOME) is never dropped.
      acceptPolicy: SignatureAcceptPolicy.acceptAll,
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
  print('║ Health scenario : ${_healthScenario.name}║');
  print('╚══════════════════════════════════════╝');
  print('Silvus radio: dart run bin/mock_radio_server.dart');

  await _startGstreamerVideo();

  _socket.listen((RawSocketEvent event) {
    if (event != RawSocketEvent.read) return;
    final dg = _socket.receive();
    if (dg == null) return;
    _parser.parse(dg.data);
  });

  _timers.add(Timer.periodic(const Duration(seconds: 1), (_) {
    _sendCompHeartbeat();
  }));

  _timers.add(Timer.periodic(const Duration(seconds: 1), (_) {
    _tickUgvSystemInfo();
    _send(_buildUgvSystemInfo());
  }));

  // ICD 5.1.7.2.2: COMP_SYSTEM_TIME is one-time at startup.
  _sendSystemTime();

  // ICD 5.1.7.2.3 / 5.1.7.2.4: GPS_RAW_INT + ATTITUDE at 10 Hz.
  _timers.add(Timer.periodic(const Duration(milliseconds: 100), (_) {
    _tickGpsMotion();
    _sendGpsRawInt();
    _sendAttitude();
  }));

  print('\nServer running. Press Ctrl+C to stop.\n');

  final stop = Completer<void>();
  void requestStop(ProcessSignal signal) {
    if (_shuttingDown) {
      print('\n[$signal] Force exit');
      exit(1);
    }
    _shuttingDown = true;
    print('\n[$signal] Shutting down…');
    if (!stop.isCompleted) stop.complete();
  }

  ProcessSignal.sigint.watch().listen(requestStop);
  if (!Platform.isWindows) {
    ProcessSignal.sigterm.watch().listen(requestStop);
  }

  await stop.future;
  await _shutdown();
}

Future<void> _shutdown() async {
  for (final t in _timers) {
    t.cancel();
  }
  _timers.clear();
  try {
    _socket.close();
  } catch (_) {}
  await _stopGstreamerVideo();
  print('Stopped.');
  exit(0);
}

Future<void> _startGstreamerVideo() async {
  final host = ServerConfig.videoStreamHost;
  final port = ServerConfig.videoStreamPort;
  // No -v: verbose gst output can fill pipes and stall the process group.
  final args = <String>[
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
    final process = await Process.start(
      'gst-launch-1.0',
      args,
      // Keep child out of the terminal process group so Ctrl+C hits Dart only.
      mode: ProcessStartMode.normal,
    );
    _gstreamerProcess = process;
    print('[GST] H.264 RTP test pattern → udp://$host:$port (pid=${process.pid})');

    // Drain pipes so gst never blocks on a full stdout/stderr buffer.
    process.stdout.drain<void>();
    process.stderr.drain<void>();
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
  process.kill(ProcessSignal.sigkill);
  try {
    await process.exitCode.timeout(const Duration(milliseconds: 500));
  } on TimeoutException {
    // already killed
  }
}

void _handleIncomingFrame(MavlinkFrame frame) {
  final msg = frame.message;
  _logMavlink(
    direction: 'RX',
    message: msg,
    systemId: frame.systemId,
    componentId: frame.componentId,
    sequence: frame.sequence,
  );

  if (msg is Heartbeat) {
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
    return;
  }

  print('[RX] Unhandled message type: ${msg.runtimeType}');
}

void _respondToTimesync(Timesync request) {
  final vehTime = _vehicleTimeUtc();
  final tc1 = vehTime.microsecondsSinceEpoch * 1000;

  final response = Timesync(
    ts1: request.ts1,
    tc1: tc1,
    targetSystem: gcsSysId,
    targetComponent: gcsCompId,
  );

  _send(response);
}

void _handleCommandLong(CommandLong cmd, MavlinkFrame frame) {
  if (cmd.targetSystem != vehicleSysId || cmd.targetComponent != vehicleCompId) {
    print(
      '[CMD] ignored — want $vehicleSysId/$vehicleCompId '
      '(got ${cmd.targetSystem}/${cmd.targetComponent})',
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
        _send(_buildUgvSystemInfo());
      } else {
        print('[CMD] SET_HOME ignored — param1=$p1 (need 1)');
      }
    case cmdOverrideSafety:
      // ICD 5.1.6.2.13: enter arm override mode
      s.armMode = 3;
      print('[CMD] OVERRIDE_SAFETY armMode=3 (Override)');
    case cmdRth:
      // ICD 5.1.6.2.14: accept RTH (no mission engine in mock)
      print('[CMD] RTH accepted (no path following in mock)');
    default:
      print('[CMD] unhandled cmd=${cmd.command}');
  }
}

void _sendCompHeartbeat() {
  // COMP_HEARTBEAT to GCS (ICD 5.1.6.2.1): base_mode=0, custom_mode=0
  _tickCount++;
  _send(Heartbeat(
    type: 18,
    autopilot: 8,
    baseMode: 0,
    customMode: 0,
    systemStatus: 4,
    mavlinkVersion: 3,
  ));
}

void _sendSystemTime() {
  final vehTime = _vehicleTimeUtc();
  final bootMs = DateTime.now().difference(_bootTime).inMilliseconds;

  _send(SystemTime(
    timeUnixUsec: BigInt.from(vehTime.microsecondsSinceEpoch),
    timeBootMs: bootMs,
  ));
}

/// Advance square GPS path + COG at 1 Hz around fixed [orbitLat]/[orbitLon].
/// Home (SET_HOME) is independent — latching home must not move the GPS path.
void _tickGpsMotion() {
  final s = _state;
  final half = gpsSquareHalfSideM;
  final sideLen = 2 * half;
  final perimeter = 4 * sideLen;

  s.gpsPathDistanceM =
      (s.gpsPathDistanceM + gpsSquareSpeedMs * gpsTickSeconds) % perimeter;

  // Sides CCW from NW corner: east → south → west → north
  final side = (s.gpsPathDistanceM / sideLen).floor();
  final along = s.gpsPathDistanceM - side * sideLen;

  late final double northM;
  late final double eastM;
  late final double cogDeg;
  switch (side) {
    case 0: // north edge, west → east
      northM = half;
      eastM = -half + along;
      cogDeg = 90;
    case 1: // east edge, north → south
      northM = half - along;
      eastM = half;
      cogDeg = 180;
    case 2: // south edge, east → west
      northM = -half;
      eastM = half - along;
      cogDeg = 270;
    default: // west edge, south → north
      northM = -half + along;
      eastM = -half;
      cogDeg = 0;
  }

  final orbitLatDeg = orbitLat / 1e7;
  final orbitLonDeg = orbitLon / 1e7;
  final metersPerDegLon =
      metersPerDegLat * math.cos(orbitLatDeg * math.pi / 180);

  final latDeg = orbitLatDeg + northM / metersPerDegLat;
  final lonDeg = orbitLonDeg + eastM / metersPerDegLon;

  s.gpsLatE7 = (latDeg * 1e7).round();
  s.gpsLonE7 = (lonDeg * 1e7).round();
  s.gpsHeadingCd = (cogDeg * 100).round() % 36000;
  s.gpsSpeedCms = (gpsSquareSpeedMs * 100).round(); // cm/s
  s.yawRad = cogDeg * math.pi / 180;
}

void _sendGpsRawInt() {
  final vehTime = _vehicleTimeUtc();
  final s = _state;

  _send(GpsRawInt(
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
  ));
}

void _sendAttitude() {
  final bootMs = DateTime.now().difference(_bootTime).inMilliseconds;
  final s = _state;

  _send(Attitude(
    timeBootMs: bootMs,
    roll: s.rollRad,
    pitch: s.pitchRad,
    yaw: s.yawRad,
    rollspeed: s.rollSpeedRad,
    pitchspeed: 0,
    yawspeed: 0,
  ));
}

DateTime _vehicleTimeUtc() {
  return DateTime.now().toUtc().add(
    const Duration(milliseconds: vehicleClockOffsetMs),
  );
}

// ── Build UGV_SYSTEM_INFO per ICD v1.4 + ugvcustom.xml ────────────────────
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

  // ICD sensor_subsystem_health_2: UHF connection, UHF link health, RGBD camera
  final sensorSubsystemHealth2 = _packFields(
    [s.uhfConnection, s.uhfLinkHealth, icdHealthy],
    [2, 2, 2],
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
bool _isHighRateMessage(MavlinkMessage message) {
  return message is GpsRawInt ||
      message is Attitude ||
      message is ManualControl;
}

bool _shouldLogMessage(MavlinkMessage message) {
  if (!_isHighRateMessage(message)) return true;
  final now = DateTime.now();
  final last = _lastHighRateLogAt[message.runtimeType];
  if (last != null && now.difference(last) < _highRateLogInterval) {
    return false;
  }
  _lastHighRateLogAt[message.runtimeType] = now;
  return true;
}

String _formatMessageData(MavlinkMessage message) {
  return jsonEncode(
    message.toJson(),
    toEncodable: (value) {
      if (value is BigInt) return value.toString();
      return value;
    },
  );
}

void _logMavlink({
  required String direction,
  required MavlinkMessage message,
  required int systemId,
  required int componentId,
  required int sequence,
  int? bytes,
}) {
  if (!_shouldLogMessage(message)) return;
  final gated = _isHighRateMessage(message) ? ' (1Hz gated)' : '';
  final size = bytes != null ? ' $bytes bytes' : '';
  print(
    '[$direction] ${message.runtimeType} id=${message.mavlinkMessageId} '
    'seq=$sequence sys=$systemId/$componentId$size$gated '
    '${_formatMessageData(message)}',
  );
}

int _send(MavlinkMessage message) {
  try {
    final seq = _sequence++ & 0xFF;
    final frame = MavlinkFrame.v2(
      seq,
      vehicleSysId,
      vehicleCompId,
      message,
      signatureManager: _signatureManager,
    );
    final bytes = frame.serialize();
    final sent = _socket.send(bytes.buffer.asUint8List(), _gcsAddress, gcsPort);
    _logMavlink(
      direction: 'TX',
      message: message,
      systemId: vehicleSysId,
      componentId: vehicleCompId,
      sequence: seq,
      bytes: sent,
    );
    return sent;
  } catch (e) {
    print('[ERR] Send failed: $e');
    return -1;
  }
}

// ── Mock state evolution (health scenario + mild telemetry drift) ─────────
void _tickUgvSystemInfo() {
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
  // gpsSpeedCms / lat-lon are owned by _tickGpsMotion (slow square path)
  s.rollSpeedRad = 0.0;

  // Slowly drain batteries
  s.lvBatterySoc = (s.lvBatterySoc > 20) ? s.lvBatterySoc - 1 : 85;
  s.hvBatterySoc = (s.hvBatterySoc > 15) ? s.hvBatterySoc - 1 : 90;

  // Gentle attitude drift for IMU telemetry
  s.yawRad = (s.yawRad + 0.002) % (2 * math.pi);
  s.rollRad = 0.02 * math.sin(_tickCount * 0.1);
  s.pitchRad = -0.01 * math.cos(_tickCount * 0.1);

  if (_healthScenario == HealthScenario.cycle) {
    _applyCycleScenario(s);
  } else {
    // Locked Polaris Connect scenarios: freeze mode/arm for predictable POST.
    s.autonomyMode = 1;
    s.holdState = 1;
    s.armMode = 2;
    s.driveLimit = 1;
    s.driveMode = 1;
    s.vcuOpState = vcuDrive;

    switch (_healthScenario) {
      case HealthScenario.success:
        // Baseline already healthy.
        break;
      case HealthScenario.partial:
        _applyPartialFailure(s);
        break;
      case HealthScenario.failure:
        _applyCompleteFailure(s);
        break;
      case HealthScenario.cycle:
        break;
    }
  }
}

/// Figma partial error: Battery ✓, Motor ✗, Sensor ✓.
void _applyPartialFailure(_MockUgvState s) {
  s.fwdLeftMotorHealth = icdFaulty;
  s.motorFaults = _packMotorFaults(
    aftPort: 0,
    aftStbd: 0,
    fwdPort: 0x20,
    fwdStbd: 0,
  );
}

/// Figma complete error: Battery ✗, Motor ✗, Sensor ✗.
void _applyCompleteFailure(_MockUgvState s) {
  s.hvBatteryHealth = icdFaulty;
  s.hvBatteryFaults = 1 << 3;

  s.fwdMcHealth = icdFaulty;
  s.mcFaults1 = 1 << 3;
  s.fwdLeftMotorHealth = icdFaulty;
  s.motorFaults = _packMotorFaults(
    aftPort: 0,
    aftStbd: 0,
    fwdPort: 0x20,
    fwdStbd: 0,
  );

  s.gnssState = icdFaulty;
  s.cameraHealthWord = 0x55A5; // port camera faulty
}

/// Legacy 30 s rotating mode + fault phases.
void _applyCycleScenario(_MockUgvState s) {
  final phase = _tickCount % 30;

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
