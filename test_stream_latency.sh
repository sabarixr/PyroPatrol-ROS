#!/bin/bash
# Stream latency test script
echo "📊 Testing streaming latency..."
echo "==============================="

# Test network latency to localhost
echo "🌐 Network latency to localhost:"
ping -c 3 localhost | tail -1 | awk -F'/' '{print "Average: " $4 "ms"}'

# Test HTTP response time
echo ""
echo "⚡ HTTP response time test:"
time curl -s http://localhost:8080/ > /dev/null && echo "✅ HTTP server responsive"

# Monitor streaming performance
echo ""
echo "📹 Monitor stream performance with:"
echo "   curl -s http://localhost:8080/stream.mjpg | pv > /dev/null"
echo ""
echo "📈 Monitor system resources:"
echo "   htop -d 1"
echo ""
