import 'dart:async';
import 'dart:convert';

import 'package:web_socket_channel/web_socket_channel.dart';

import '../models/robot_telemetry.dart';

class RobotSocketService {
  WebSocketChannel? _channel;
  StreamSubscription? _subscription;
  final StreamController<RobotTelemetry> _telemetryController =
      StreamController<RobotTelemetry>.broadcast();

  RobotTelemetry _current = const RobotTelemetry();

  Stream<RobotTelemetry> get telemetryStream => _telemetryController.stream;

  Future<void> connect({
    required String host,
    required int port,
  }) async {
    await disconnect();

    final uri = Uri.parse('ws://$host:$port');
    _channel = WebSocketChannel.connect(uri);

    _current = _current.copyWith(
      connected: true,
      status: 'Connected to $host:$port',
      lastMessage: '',
    );
    _telemetryController.add(_current);

    _subscription = _channel!.stream.listen(
      _handleMessage,
      onError: (Object error) {
        _emitDisconnected('Socket error: $error');
      },
      onDone: () {
        _emitDisconnected('Socket closed');
      },
      cancelOnError: true,
    );
  }

  Future<void> disconnect() async {
    await _subscription?.cancel();
    _subscription = null;
    await _channel?.sink.close();
    _channel = null;
    _emitDisconnected('Disconnected');
  }

  void dispose() {
    disconnect();
    _telemetryController.close();
  }

  void sendTwist({required double linearX, required double angularZ}) {
    sendJson({
      'topic': '/cmd_vel',
      'twist': {
        'linear': {'x': linearX, 'y': 0, 'z': 0},
        'angular': {'x': 0, 'y': 0, 'z': angularZ},
      },
    });
  }

  void sendTilt(double angle) {
    sendJson({'tilt': angle});
  }

  void sendEsp32Command(String command) {
    sendJson({'topic': '/esp32_command', 'data': command});
  }

  void sendMissionCommand(String command) {
    sendJson({'cmd': command});
  }

  void stop() => sendEsp32Command('STOP');

  void scan() => sendEsp32Command('SCAN');

  void togglePump(bool on) => sendEsp32Command(on ? 'PUMP_ON' : 'PUMP_OFF');

  void sendJson(Map<String, Object?> payload) {
    _channel?.sink.add(jsonEncode(payload));
  }

  void _handleMessage(dynamic message) {
    final raw = message.toString();
    try {
      final decoded = jsonDecode(raw);
      if (decoded is Map<String, dynamic>) {
        _current = _mergeTelemetry(decoded, raw);
        _telemetryController.add(_current);
        return;
      }
    } catch (_) {
      // Fall through to plain-text status updates.
    }

    _current = _current.copyWith(lastMessage: raw, status: raw);
    _telemetryController.add(_current);
  }

  RobotTelemetry _mergeTelemetry(Map<String, dynamic> data, String raw) {
    var next = _current.copyWith(lastMessage: raw);
    final type = data['type']?.toString() ?? '';

    if (data.containsKey('mq2')) {
      next = next.copyWith(mq2: _asInt(data['mq2']));
    }
    if (data.containsKey('mq5')) {
      next = next.copyWith(mq5: _asInt(data['mq5']));
    }
    if (data.containsKey('temp')) {
      next = next.copyWith(temperature: _asDouble(data['temp']));
    }
    if (data.containsKey('pump_active')) {
      next = next.copyWith(pumpActive: _asBool(data['pump_active']));
    }
    if (data.containsKey('direction')) {
      next = next.copyWith(scanDirection: data['direction'].toString());
    }
    if (data.containsKey('msg')) {
      next = next.copyWith(status: data['msg'].toString());
    }

    if (type == 'pump') {
      next = next.copyWith(pumpActive: data['status'] == 'on');
    } else if (type == 'alert') {
      next = next.copyWith(
        fireDetected: true,
        status: data['msg']?.toString() ?? 'Fire alert',
      );
    } else if (type == 'scan_complete') {
      next = next.copyWith(
        scanDirection: data['direction']?.toString() ?? '',
        status: 'Scan complete',
      );
    } else if (type == 'system' && data['state'] != null) {
      next = next.copyWith(status: 'ESP32 state: ${data['state']}');
    }

    return next.copyWith(connected: true);
  }

  void _emitDisconnected(String status) {
    _current = _current.copyWith(connected: false, status: status);
    _telemetryController.add(_current);
  }

  int _asInt(Object? value) {
    if (value is int) {
      return value;
    }
    if (value is double) {
      return value.round();
    }
    return int.tryParse(value.toString()) ?? 0;
  }

  double _asDouble(Object? value) {
    if (value is double) {
      return value;
    }
    if (value is int) {
      return value.toDouble();
    }
    return double.tryParse(value.toString()) ?? 0;
  }

  bool _asBool(Object? value) {
    if (value is bool) {
      return value;
    }
    return value.toString().toLowerCase() == 'true';
  }
}
