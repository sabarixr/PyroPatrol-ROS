import 'package:flutter_test/flutter_test.dart';

import 'package:pyropatrol_rover_control/main.dart';

void main() {
  testWidgets('shows rover app connect screen', (WidgetTester tester) async {
    await tester.pumpWidget(const RiderApp());

    expect(find.text('PyroPatrol Rover Control'), findsOneWidget);
    expect(find.text('Connect'), findsOneWidget);
  });
}
