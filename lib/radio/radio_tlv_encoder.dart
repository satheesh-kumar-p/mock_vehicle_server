import 'dart:convert';
import 'dart:typed_data';

import 'radio_simulator.dart';

/// Encodes Silvus TLV UDP reports consumed by scout-td0-GCS RadioTlvParser.
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

  Uint8List encodeRssiReport(SimulatedRadioSample sample) {
    final builder = BytesBuilder();
    _appendTlv(builder, typeRssiStartOfReport, '');
    _appendTlv(builder, typeRssiRevision, '1.0');
    _appendTlv(builder, typeSequenceNumber, '${sample.sequenceNumber}');
    _appendTlv(builder, typeAntenna1, '${sample.antenna1Dbm}');
    _appendTlv(builder, typeAntenna2, '${sample.antenna2Dbm}');
    _appendTlv(builder, typeAntenna3, '${sample.antenna3Dbm}');
    _appendTlv(builder, typeAntenna4, '${sample.antenna4Dbm}');
    _appendTlv(builder, typeNoise, '${sample.noiseDbm}');
    _appendTlv(builder, typeSyncSignal, '${sample.syncSignal}');
    _appendTlv(builder, typeSyncNoise, '${sample.syncNoise}');
    _appendTlv(builder, typeNodeId, '${sample.nodeId}');
    _appendTlv(builder, typeEndOfReport, '');
    return builder.toBytes();
  }

  Uint8List encodeTemperatureReport(SimulatedRadioSample sample) {
    final builder = BytesBuilder();
    _appendTlv(builder, typeTempStartOfReport, '');
    _appendTlv(builder, typeTempRevision, '1.0');
    _appendTlv(builder, typeTempCurrent, '${sample.temperatureCelsius}');
    _appendTlv(builder, typeTempMax, '${sample.maxTemperatureCelsius}');
    _appendTlv(builder, typeTempOverheatCount, '${sample.overheatCount}');
    _appendTlv(builder, typeEndOfReport, '');
    return builder.toBytes();
  }

  void _appendTlv(BytesBuilder builder, int type, String value) {
    final valueBytes = utf8.encode(value);
    final header = ByteData(4);
    header.setUint16(0, type, Endian.big);
    header.setUint16(2, valueBytes.length, Endian.big);
    builder
      ..add(header.buffer.asUint8List())
      ..add(valueBytes);
  }
}
