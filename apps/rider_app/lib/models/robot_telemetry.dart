class RobotTelemetry {
  const RobotTelemetry({
    this.connected = false,
    this.mq2 = 0,
    this.mq5 = 0,
    this.temperature = 25,
    this.pumpActive = false,
    this.fireDetected = false,
    this.scanDirection = '',
    this.status = 'Disconnected',
    this.lastMessage = '',
  });

  final bool connected;
  final int mq2;
  final int mq5;
  final double temperature;
  final bool pumpActive;
  final bool fireDetected;
  final String scanDirection;
  final String status;
  final String lastMessage;

  RobotTelemetry copyWith({
    bool? connected,
    int? mq2,
    int? mq5,
    double? temperature,
    bool? pumpActive,
    bool? fireDetected,
    String? scanDirection,
    String? status,
    String? lastMessage,
  }) {
    return RobotTelemetry(
      connected: connected ?? this.connected,
      mq2: mq2 ?? this.mq2,
      mq5: mq5 ?? this.mq5,
      temperature: temperature ?? this.temperature,
      pumpActive: pumpActive ?? this.pumpActive,
      fireDetected: fireDetected ?? this.fireDetected,
      scanDirection: scanDirection ?? this.scanDirection,
      status: status ?? this.status,
      lastMessage: lastMessage ?? this.lastMessage,
    );
  }
}
