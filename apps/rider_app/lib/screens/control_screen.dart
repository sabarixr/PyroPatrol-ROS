import 'dart:async';

import 'package:flutter/material.dart';
import 'package:flutter_joystick/flutter_joystick.dart';
import 'package:webview_flutter/webview_flutter.dart';

import '../models/robot_telemetry.dart';
import '../services/robot_socket_service.dart';

class ControlScreen extends StatefulWidget {
  const ControlScreen({
    super.key,
    required this.robotIp,
    required this.wsPort,
    required this.videoPort,
  });

  final String robotIp;
  final int wsPort;
  final int videoPort;

  @override
  State<ControlScreen> createState() => _ControlScreenState();
}

class _ControlScreenState extends State<ControlScreen> {
  final RobotSocketService _socketService = RobotSocketService();
  late final WebViewController _webViewController;

  StreamSubscription<RobotTelemetry>? _telemetrySubscription;
  RobotTelemetry _telemetry = const RobotTelemetry();
  bool _pumpActive = false;
  double _tilt = 90;

  @override
  void initState() {
    super.initState();
    _webViewController = WebViewController()
      ..setJavaScriptMode(JavaScriptMode.unrestricted)
      ..loadRequest(
        Uri.parse('http://${widget.robotIp}:${widget.videoPort}'),
      );

    _telemetrySubscription = _socketService.telemetryStream.listen((telemetry) {
      if (!mounted) {
        return;
      }
      setState(() {
        _telemetry = telemetry;
        _pumpActive = telemetry.pumpActive;
      });
    });

    unawaited(
      _socketService.connect(host: widget.robotIp, port: widget.wsPort),
    );
  }

  @override
  void dispose() {
    _telemetrySubscription?.cancel();
    _socketService.dispose();
    super.dispose();
  }

  void _sendJoystick(StickDragDetails details) {
    final linear = (-details.y).clamp(-1.0, 1.0).toDouble();
    final angular = details.x.clamp(-1.0, 1.0).toDouble();
    _socketService.sendTwist(linearX: linear, angularZ: angular);
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: Text('Rover Control ${widget.robotIp}'),
        actions: [
          Padding(
            padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 10),
            child: DecoratedBox(
              decoration: BoxDecoration(
                color: _telemetry.connected ? Colors.green : Colors.red,
                borderRadius: BorderRadius.circular(999),
              ),
              child: Padding(
                padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 8),
                child: Text(_telemetry.connected ? 'Connected' : 'Offline'),
              ),
            ),
          ),
        ],
      ),
      body: Row(
        children: [
          Expanded(
            flex: 3,
            child: Padding(
              padding: const EdgeInsets.all(12),
              child: ClipRRect(
                borderRadius: BorderRadius.circular(16),
                child: WebViewWidget(controller: _webViewController),
              ),
            ),
          ),
          Expanded(
            flex: 2,
            child: Padding(
              padding: const EdgeInsets.fromLTRB(0, 12, 12, 12),
              child: Column(
                children: [
                  _StatusPanel(telemetry: _telemetry),
                  const SizedBox(height: 12),
                  Expanded(
                    child: Row(
                      children: [
                        Expanded(
                          child: Card(
                            child: Center(
                              child: Joystick(
                                mode: JoystickMode.all,
                                listener: _sendJoystick,
                              ),
                            ),
                          ),
                        ),
                        const SizedBox(width: 12),
                        Expanded(
                          child: _ActionPanel(
                            pumpActive: _pumpActive,
                            tilt: _tilt,
                            onStop: _socketService.stop,
                            onScan: _socketService.scan,
                            onPumpToggle: () {
                              final next = !_pumpActive;
                              setState(() {
                                _pumpActive = next;
                              });
                              _socketService.togglePump(next);
                            },
                            onManualMode: () {
                              _socketService.sendMissionCommand('manual_mode');
                            },
                            onFireMode: () {
                              _socketService.sendMissionCommand('start_fire_mode');
                            },
                            onArucoMode: () {
                              _socketService.sendMissionCommand('follow_aruco');
                            },
                            onTiltChanged: (value) {
                              setState(() {
                                _tilt = value;
                              });
                              _socketService.sendTilt(value);
                            },
                          ),
                        ),
                      ],
                    ),
                  ),
                ],
              ),
            ),
          ),
        ],
      ),
    );
  }
}

class _StatusPanel extends StatelessWidget {
  const _StatusPanel({required this.telemetry});

  final RobotTelemetry telemetry;

  @override
  Widget build(BuildContext context) {
    return Card(
      child: Padding(
        padding: const EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            const Text(
              'Telemetry',
              style: TextStyle(fontSize: 18, fontWeight: FontWeight.w700),
            ),
            const SizedBox(height: 12),
            Wrap(
              spacing: 8,
              runSpacing: 8,
              children: [
                _MetricChip(label: 'MQ2', value: telemetry.mq2.toString()),
                _MetricChip(label: 'MQ5', value: telemetry.mq5.toString()),
                _MetricChip(
                  label: 'Temp',
                  value: '${telemetry.temperature.toStringAsFixed(1)} C',
                ),
                _MetricChip(
                  label: 'Pump',
                  value: telemetry.pumpActive ? 'On' : 'Off',
                ),
                _MetricChip(
                  label: 'Fire',
                  value: telemetry.fireDetected ? 'Detected' : 'Clear',
                ),
                _MetricChip(
                  label: 'Scan',
                  value: telemetry.scanDirection.isEmpty
                      ? 'Idle'
                      : telemetry.scanDirection,
                ),
              ],
            ),
            const SizedBox(height: 12),
            Text('Status: ${telemetry.status}'),
            const SizedBox(height: 6),
            Text(
              telemetry.lastMessage.isEmpty
                  ? 'Waiting for telemetry...'
                  : telemetry.lastMessage,
              maxLines: 3,
              overflow: TextOverflow.ellipsis,
              style: TextStyle(
                color: Theme.of(context).colorScheme.onSurfaceVariant,
              ),
            ),
          ],
        ),
      ),
    );
  }
}

class _ActionPanel extends StatelessWidget {
  const _ActionPanel({
    required this.pumpActive,
    required this.tilt,
    required this.onStop,
    required this.onScan,
    required this.onPumpToggle,
    required this.onManualMode,
    required this.onFireMode,
    required this.onArucoMode,
    required this.onTiltChanged,
  });

  final bool pumpActive;
  final double tilt;
  final VoidCallback onStop;
  final VoidCallback onScan;
  final VoidCallback onPumpToggle;
  final VoidCallback onManualMode;
  final VoidCallback onFireMode;
  final VoidCallback onArucoMode;
  final ValueChanged<double> onTiltChanged;

  @override
  Widget build(BuildContext context) {
    return Card(
      child: Padding(
        padding: const EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.stretch,
          children: [
            FilledButton(
              style: FilledButton.styleFrom(backgroundColor: Colors.red),
              onPressed: onStop,
              child: const Text('Stop'),
            ),
            const SizedBox(height: 8),
            FilledButton(
              onPressed: onScan,
              child: const Text('Scan'),
            ),
            const SizedBox(height: 8),
            FilledButton(
              onPressed: onPumpToggle,
              child: Text(pumpActive ? 'Pump Off' : 'Pump On'),
            ),
            const SizedBox(height: 16),
            const Text(
              'Modes',
              style: TextStyle(fontSize: 16, fontWeight: FontWeight.w700),
            ),
            const SizedBox(height: 8),
            OutlinedButton(
              onPressed: onManualMode,
              child: const Text('Manual'),
            ),
            OutlinedButton(
              onPressed: onFireMode,
              child: const Text('Fire Seek'),
            ),
            OutlinedButton(
              onPressed: onArucoMode,
              child: const Text('Follow ArUco'),
            ),
            const Spacer(),
            Text('Camera Tilt: ${tilt.round()}'),
            Slider(
              min: 0,
              max: 180,
              value: tilt,
              onChanged: onTiltChanged,
            ),
          ],
        ),
      ),
    );
  }
}

class _MetricChip extends StatelessWidget {
  const _MetricChip({required this.label, required this.value});

  final String label;
  final String value;

  @override
  Widget build(BuildContext context) {
    return Chip(
      label: Text('$label: $value'),
      backgroundColor: Theme.of(context).colorScheme.surfaceContainerHighest,
    );
  }
}
