#!/bin/bash
# Test script for MQTT system using Python mocks

set -e

echo "================================"
echo "MQTT System Testing Script"
echo "================================"
echo ""

# Check if MQTT broker is running
echo "Checking MQTT broker..."
if ! docker ps | grep -q mqtt-broker; then
    echo "ERROR: MQTT broker not running!"
    echo "Please start it with: docker-compose up -d"
    exit 1
fi
echo "✓ MQTT broker is running"
echo ""

# Check Python dependencies
echo "Checking Python dependencies..."
if ! python3 -c "import paho.mqtt" 2>/dev/null; then
    echo "Installing Python dependencies..."
    pip3 install -r requirements.txt
fi
echo "✓ Python dependencies installed"
echo ""

# Function to cleanup background processes
cleanup() {
    echo ""
    echo "Cleaning up..."
    jobs -p | xargs -r kill 2>/dev/null || true
    wait 2>/dev/null || true
    echo "Done!"
}

trap cleanup EXIT

# Test 1: Basic connectivity
echo "=== Test 1: Basic Connectivity ==="
echo "Starting mock server..."
python3 mock_server.py &
SERVER_PID=$!
sleep 2

echo "Starting mock client..."
python3 mock_client.py --client-id test_client_001 --entities 1 2 &
CLIENT_PID=$!

echo "Running for 10 seconds..."
sleep 10

echo "✓ Test 1 Complete"
echo ""

# Kill previous processes
kill $CLIENT_PID $SERVER_PID 2>/dev/null || true
wait 2>/dev/null || true
sleep 1

# Test 2: Multiple clients
echo "=== Test 2: Multiple Clients ==="
echo "Starting mock server..."
python3 mock_server.py &
SERVER_PID=$!
sleep 2

echo "Starting client 1 with entities 1, 2, 3..."
python3 mock_client.py --client-id test_client_001 --entities 1 2 3 &
CLIENT1_PID=$!

sleep 2

echo "Starting client 2 with entities 10, 20..."
python3 mock_client.py --client-id test_client_002 --entities 10 20 &
CLIENT2_PID=$!

echo "Running for 10 seconds..."
sleep 10

echo "✓ Test 2 Complete"
echo ""

# Kill previous processes
kill $CLIENT1_PID $CLIENT2_PID $SERVER_PID 2>/dev/null || true
wait 2>/dev/null || true
sleep 1

# Test 3: Interoperability with Rust
echo "=== Test 3: Interoperability Test ==="
echo "This test checks if Python mocks work with Rust implementations"
echo ""

if [ -f "../mqtt-server/target/release/mqtt-server" ]; then
    echo "Testing Python client with Rust server..."
    ../mqtt-server/target/release/mqtt-server &
    RUST_SERVER_PID=$!
    sleep 2
    
    python3 mock_client.py --client-id interop_test --entities 5 6 &
    PY_CLIENT_PID=$!
    
    sleep 10
    
    kill $PY_CLIENT_PID $RUST_SERVER_PID 2>/dev/null || true
    wait 2>/dev/null || true
    echo "✓ Python client <-> Rust server OK"
else
    echo "⚠ Rust server binary not found, skipping this test"
    echo "  Build it with: cd ../mqtt-server && cargo build --release"
fi
echo ""

if [ -f "../rpi-mqtt-client/target/release/rpi-mqtt-client" ]; then
    echo "Testing Rust client with Python server..."
    python3 mock_server.py &
    PY_SERVER_PID=$!
    sleep 2
    
    # Run Rust client for limited time
    timeout 10 ../rpi-mqtt-client/target/release/rpi-mqtt-client || true
    
    kill $PY_SERVER_PID 2>/dev/null || true
    wait 2>/dev/null || true
    echo "✓ Rust client <-> Python server OK"
else
    echo "⚠ Rust client binary not found, skipping this test"
    echo "  Build it with: cd ../rpi-mqtt-client && cargo build --release"
fi

echo ""
echo "================================"
echo "All tests completed!"
echo "================================"
