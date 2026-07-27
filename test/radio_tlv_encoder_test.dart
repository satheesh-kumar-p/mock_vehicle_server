import 'dart:typed_data';

import 'package:flutter_test/flutter_test.dart';
import 'package:mock_vehicle_server/radio/radio_simulator.dart';
import 'package:mock_vehicle_server/radio/radio_tlv_encoder.dart';

void main() {
  const encoder = RadioTlvEncoder();

  group('§5 Table 2 RSSI report', () {
    test('manual sample encodes to exactly 109 bytes', () {
      final bytes = encoder.encodeRssiReportFromManualSample();
      expect(bytes.length, 109);
    });

    test('matches reconstructed StreamCaster hex dump', () {
      final bytes = encoder.encodeRssiReportFromManualSample();
      final expected = _hexToBytes('''
13 91 00 01 00
13 92 00 04 31 2e 30 00
13 90 00 05 32 33 33 33 00
13 88 00 05 20 2d 34 33 00
13 89 00 05 20 2d 33 31 00
13 8a 00 05 20 2d 32 38 00
13 8b 00 05 20 2d 36 36 00
13 8c 00 05 2d 31 39 30 00
13 8d 00 0a 20 20 38 36 30 34 35 36 38 00
13 8e 00 0a 20 20 38 38 36 31 33 32 32 00
13 8f 00 05 31 30 32 35 00
00 01 00 01 00
''');
      expect(bytes, expected);
    });

    test('field order and null-terminated lengths', () {
      final bytes = encoder.encodeRssiReportFromManualSample();
      final fields = _parseTlvs(bytes);

      expect(fields.map((f) => f.type).toList(), [
        5009,
        5010,
        5008,
        5000,
        5001,
        5002,
        5003,
        5004,
        5005,
        5006,
        5007,
        1,
      ]);
      expect(fields.first.value, '');
      expect(fields[1].value, '1.0');
      expect(fields[2].value, '2333');
      expect(fields[3].value, ' -43');
      expect(fields[8].value, '  8604568');
      expect(fields.last.type, 1);
      expect(fields.last.value, '');
      for (final field in fields) {
        expect(field.lengthIncludesNull, isTrue);
      }
    });
  });

  group('§6 temperature report', () {
    test('types 8 → 9 → 2 → 3 → 4 → 1 with null-terminated values', () {
      final sample = SimulatedRadioSample.manualTable2;
      final bytes = encoder.encodeTemperatureReport(sample);
      final fields = _parseTlvs(bytes);

      expect(fields.map((f) => f.type).toList(), [8, 9, 2, 3, 4, 1]);
      expect(fields[0].value, '');
      expect(fields[1].value, '1.0');
      expect(fields[2].value.trim(), '45');
      expect(fields[3].value.trim(), '55');
      expect(fields[4].value.trim(), '0');
      expect(fields[5].value, '');
      for (final field in fields) {
        expect(field.lengthIncludesNull, isTrue);
        expect(field.rawValue.last, 0);
      }
    });
  });
}

class _TlvField {
  _TlvField({
    required this.type,
    required this.length,
    required this.rawValue,
  });

  final int type;
  final int length;
  final Uint8List rawValue;

  String get value =>
      String.fromCharCodes(rawValue.sublist(0, rawValue.length - 1));

  bool get lengthIncludesNull => length == rawValue.length;
}

List<_TlvField> _parseTlvs(Uint8List bytes) {
  final fields = <_TlvField>[];
  var offset = 0;
  final data = ByteData.sublistView(bytes);
  while (offset + 4 <= bytes.length) {
    final type = data.getUint16(offset, Endian.big);
    final length = data.getUint16(offset + 2, Endian.big);
    offset += 4;
    expect(offset + length, lessThanOrEqualTo(bytes.length));
    final value = bytes.sublist(offset, offset + length);
    offset += length;
    fields.add(_TlvField(type: type, length: length, rawValue: value));
    if (type == 1) break;
  }
  return fields;
}

Uint8List _hexToBytes(String hex) {
  final parts = hex
      .split(RegExp(r'\s+'))
      .where((p) => p.isNotEmpty)
      .map((p) => int.parse(p, radix: 16))
      .toList();
  return Uint8List.fromList(parts);
}
