import 'package:flutter_test/flutter_test.dart';

import 'package:rider_app/main.dart';

void main() {
  testWidgets('shows rover app connect screen', (WidgetTester tester) async {
    await tester.pumpWidget(const RiderApp());

    expect(find.text('Rover App'), findsOneWidget);
    expect(find.text('Connect'), findsOneWidget);
  });
}
