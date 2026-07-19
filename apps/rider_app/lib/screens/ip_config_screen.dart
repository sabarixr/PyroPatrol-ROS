import 'package:flutter/material.dart';

import 'control_screen.dart';

class IPConfigScreen extends StatefulWidget {
  const IPConfigScreen({super.key});

  @override
  State<IPConfigScreen> createState() => _IPConfigScreenState();
}

class _IPConfigScreenState extends State<IPConfigScreen> {
  final TextEditingController _ipController =
      TextEditingController(text: '10.226.116.89');
  final TextEditingController _wsPortController =
      TextEditingController(text: '8765');
  final TextEditingController _videoPortController =
      TextEditingController(text: '8080');

  @override
  void dispose() {
    _ipController.dispose();
    _wsPortController.dispose();
    _videoPortController.dispose();
    super.dispose();
  }

  void _connect() {
    final ip = _ipController.text.trim();
    final wsPort = int.tryParse(_wsPortController.text.trim()) ?? 8765;
    final videoPort = int.tryParse(_videoPortController.text.trim()) ?? 8080;

    if (ip.isEmpty) {
      ScaffoldMessenger.of(context).showSnackBar(
        const SnackBar(content: Text('Enter a robot IP address.')),
      );
      return;
    }

    Navigator.of(context).pushReplacement(
      MaterialPageRoute<void>(
        builder: (_) => ControlScreen(
          robotIp: ip,
          wsPort: wsPort,
          videoPort: videoPort,
        ),
      ),
    );
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: Container(
        decoration: const BoxDecoration(
          gradient: LinearGradient(
            begin: Alignment.topLeft,
            end: Alignment.bottomRight,
            colors: [Color(0xFF181818), Color(0xFF050505)],
          ),
        ),
        child: Center(
          child: ConstrainedBox(
            constraints: const BoxConstraints(maxWidth: 560),
            child: Card(
              child: Padding(
                padding: const EdgeInsets.all(24),
                child: Column(
                  mainAxisSize: MainAxisSize.min,
                  crossAxisAlignment: CrossAxisAlignment.stretch,
                  children: [
                    const Text(
                      'PyroPatrol Rover Control',
                      textAlign: TextAlign.center,
                      style: TextStyle(
                        fontSize: 28,
                        fontWeight: FontWeight.w700,
                      ),
                    ),
                    const SizedBox(height: 8),
                    Text(
                      'Connect the control app to the robot WebSocket and camera stream.',
                      textAlign: TextAlign.center,
                      style: TextStyle(
                        color: Theme.of(context).colorScheme.onSurfaceVariant,
                      ),
                    ),
                    const SizedBox(height: 24),
                    TextField(
                      controller: _ipController,
                      decoration: const InputDecoration(
                        labelText: 'Robot IP address',
                        border: OutlineInputBorder(),
                      ),
                    ),
                    const SizedBox(height: 16),
                    Row(
                      children: [
                        Expanded(
                          child: TextField(
                            controller: _wsPortController,
                            keyboardType: TextInputType.number,
                            decoration: const InputDecoration(
                              labelText: 'WebSocket port',
                              border: OutlineInputBorder(),
                            ),
                          ),
                        ),
                        const SizedBox(width: 16),
                        Expanded(
                          child: TextField(
                            controller: _videoPortController,
                            keyboardType: TextInputType.number,
                            decoration: const InputDecoration(
                              labelText: 'Video port',
                              border: OutlineInputBorder(),
                            ),
                          ),
                        ),
                      ],
                    ),
                    const SizedBox(height: 24),
                    FilledButton(
                      onPressed: _connect,
                      child: const Padding(
                        padding: EdgeInsets.symmetric(vertical: 14),
                        child: Text('Connect'),
                      ),
                    ),
                  ],
                ),
              ),
            ),
          ),
        ),
      ),
    );
  }
}
