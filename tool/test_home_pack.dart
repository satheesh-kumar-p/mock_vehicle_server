import 'dart:typed_data';
import 'package:scout_mavlink_dart/scout_mavlink_dart.dart';

void main() {
  final msg = UgvSystemInfo(
    vcuStatus: 0,
    batterySoc: 0,
    compMode1: 0,
    compMode2: 0,
    sensorSubsystemHealth1: 0,
    sensorSubsystemHealth2: 0,
    sensorSubsystemHealth3: 0,
    sensorSubsystemHealth4: 0,
    vcuSubsystemStatus: 0,
    compSubsystemStatus: 0,
    vcuSubsystemPowerState1: 0,
    vcuPowerSubsystemState2: 0,
    motorFaults: 0,
    validityMotorFaults: 0,
    mcFaults1: 0,
    mcFaults2: 0,
    contactorFault: 0,
    pduFault: 0,
    powerSubsystemFaults1: 0,
    powerSubsystemFaults2: 0,
    vcuInterfaceHealth: 0,
    secCompStatus: 0,
    compInterfaceHealth1: 0,
    compInterfaceHealth2: 0,
    homeLocation: 1, // bit0 set
    lat: 130828500,
    lon: 776234500,
  );
  final bytes = msg.serialize();
  print('len=${bytes.lengthInBytes} homeByte=${bytes.getUint8(54)} lat=${bytes.getInt32(16, Endian.little)} lon=${bytes.getInt32(20, Endian.little)}');
  final parsed = UgvSystemInfo.parse(bytes);
  print('parsed home=${parsed.homeLocation} lat=${parsed.lat} lon=${parsed.lon}');
}
