import 'dart:convert';
import 'dart:typed_data';

import 'radio_simulator.dart';

/// Encodes Silvus TLV UDP reports per StreamCaster API Manual §§4–6.
///
/// Wire format (network endian):
/// - TYPE / LENGTH are uint16
/// - VALUE is ASCII text terminated with `\0`
/// - LENGTH includes the terminating null byte
class RadioTlvEncoder {
  const RadioTlvEncoder();

  static const int typeEndOfReport = 1;
  static const int typeTempStartOfReport = 8;
  static const int typeTempRevision = 9;
  static const int typeTempCurrent = 2;
  static const int typeTempMax = 3;
  static const int typeTempOverheatCount = 4;
  static const int typeRssiStartOfReport = 5009;
  static const int typeRssiRevision = 5010;
  static const int typeAntenna1 = 5000;
  static const int typeAntenna2 = 5001;
  static const int typeAntenna3 = 5002;
  static const int typeAntenna4 = 5003;
  static const int typeNoise = 5004;
  static const int typeSyncSignal = 5005;
  static const int typeSyncNoise = 5006;
  static const int typeNodeId = 5007;
  static const int typeSequenceNumber = 5008;

  /// Field widths matching §5 Table 2 / hex dump (excluding null).
  static const int widthShort = 4;
  static const int widthSync = 9;

  /// §5 Table 2 sample values (exact manual example).
  static const SimulatedRadioSample manualSample =
      SimulatedRadioSample.manualTable2;

  /// Encodes the exact §5 Table 2 RSSI report (109 bytes).
  Uint8List encodeRssiReportFromManualSample() {
    return encodeRssiReport(manualSample);
  }

  Uint8List encodeRssiReport(SimulatedRadioSample sample) {
    final builder = BytesBuilder();
    _appendTlv(builder, typeRssiStartOfReport, '');
    _appendTlv(builder, typeRssiRevision, '1.0');
    _appendTlv(
      builder,
      typeSequenceNumber,
      _padInt(sample.sequenceNumber, widthShort),
    );
    _appendTlv(builder, typeAntenna1, _padInt(sample.antenna1Dbm, widthShort));
    _appendTlv(builder, typeAntenna2, _padInt(sample.antenna2Dbm, widthShort));
    _appendTlv(builder, typeAntenna3, _padInt(sample.antenna3Dbm, widthShort));
    _appendTlv(builder, typeAntenna4, _padInt(sample.antenna4Dbm, widthShort));
    _appendTlv(builder, typeNoise, _padInt(sample.noiseDbm, widthShort));
    _appendTlv(
      builder,
      typeSyncSignal,
      _padInt(sample.syncSignal, widthSync),
    );
    _appendTlv(builder, typeSyncNoise, _padInt(sample.syncNoise, widthSync));
    _appendTlv(builder, typeNodeId, _padInt(sample.nodeId, widthShort));
    _appendTlv(builder, typeEndOfReport, '');
    return builder.toBytes();
  }

  /// §6 temperature report: types 8 → 9 → 2 → 3 → 4 → 1.
  Uint8List encodeTemperatureReport(SimulatedRadioSample sample) {
    final builder = BytesBuilder();
    _appendTlv(builder, typeTempStartOfReport, '');
    _appendTlv(builder, typeTempRevision, '1.0');
    _appendTlv(
      builder,
      typeTempCurrent,
      _padInt(sample.temperatureCelsius, widthShort),
    );
    _appendTlv(
      builder,
      typeTempMax,
      _padInt(sample.maxTemperatureCelsius, widthShort),
    );
    _appendTlv(
      builder,
      typeTempOverheatCount,
      _padInt(sample.overheatCount, widthShort),
    );
    _appendTlv(builder, typeEndOfReport, '');
    return builder.toBytes();
  }

  /// Left space-pads [value] to [width] characters (as in the manual hex dump).
  String _padInt(int value, int width) => '$value'.padLeft(width);

  void _appendTlv(BytesBuilder builder, int type, String value) {
    final valueBytes = utf8.encode(value);
    // LENGTH includes the terminating null byte (§4).
    final length = valueBytes.length + 1;
    final header = ByteData(4);
    header.setUint16(0, type, Endian.big);
    header.setUint16(2, length, Endian.big);
    builder
      ..add(header.buffer.asUint8List())
      ..add(valueBytes)
      ..addByte(0);
  }
}
