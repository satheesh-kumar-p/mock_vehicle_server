import 'dart:async';
import 'dart:io';
import 'dart:typed_data';

import 'package:scout_mavlink_dart/scout_mavlink_dart.dart';

final _key = Uint8List.fromList([
  0x5f, 0xb2, 0x1c, 0x3a, 0x9e, 0xd4, 0x6f, 0x0b,
  0x2c, 0x7e, 0xa1, 0x4d, 0x83, 0x5b, 0xc9, 0x3f,
  0x0e, 0x6d, 0xb8, 0x94, 0x2a, 0xf1, 0xc5, 0x76,
  0x40, 0xed, 0x99, 0x13, 0xab, 0x5c, 0xe2, 0x04,
]);

Future<void> main(List<String> args) async {
  final port = int.parse(args.isNotEmpty ? args[0] : '7500');
  final duration = Duration(seconds: int.parse(args.length > 1 ? args[1] : '5'));

  final sig = MavlinkSignatureManager(
    MavlinkSignatureConfig(
      secretKey: _key,
      linkId: 1,
      acceptPolicy: SignatureAcceptPolicy.acceptUnsigned,
    ),
  );
  final parser = MavlinkParser(MavlinkDialectUgvcustom(), signatureManager: sig);
  final counts = <String, int>{};

  parser.stream.listen((frame) {
    final name = frame.message.runtimeType.toString();
    counts[name] = (counts[name] ?? 0) + 1;
    print(
      '[RX] $name id=${frame.message.mavlinkMessageId} '
      'sys=${frame.systemId} comp=${frame.componentId}',
    );
  });

  final sock = await RawDatagramSocket.bind(InternetAddress.anyIPv4, port);
  print('Listening on UDP :$port for ${duration.inSeconds}s...');

  sock.listen((event) {
    if (event != RawSocketEvent.read) return;
    final dg = sock.receive();
    if (dg == null) return;
    parser.parse(dg.data);
  });

  await Future<void>.delayed(duration);
  sock.close();
  print('\nSummary (${counts.values.fold(0, (a, b) => a + b)} frames):');
  for (final e in counts.entries) {
    print('  ${e.key}: ${e.value}');
  }
}
