# Python Mock Testing Guide

Python implementations of the MQTT client and server for testing and development.

## Features

✅ **Binary-compatible** with Rust implementations (uses same bincode format)
✅ **Lightweight** and easy to run
✅ **Configurable** via command-line arguments
✅ **Perfect for testing** without deploying to Raspberry Pi hardware

## Installation

```bash
pip3 install -r requirements.txt
```

Dependencies:
- `paho-mqtt`: MQTT client library
- `msgpack`: Not actually used (we use struct for bincode compatibility)

## Usage

### Mock Client

Simulates a Raspberry Pi with multiple entities:

```bash
# Basic usage (3 entities by default)
python3 mock_client.py

# Custom client ID and entities
python3 mock_client.py --client-id my_robot_01 --entities 1 2 3 4 5

# Connect to remote broker
python3 mock_client.py --broker 192.168.1.100 --port 1883
```

**Arguments:**
- `--client-id`: Unique client identifier (default: `mock_rpi_001`)
- `--broker`: MQTT broker address (default: `localhost`)
- `--port`: MQTT broker port (default: `1883`)
- `--entities`: Space-separated list of entity IDs (default: `1 2 3`)

**What it does:**
- Sends heartbeat every 500ms
- Sends state reports for each entity at 10 Hz
- Receives and prints segment commands
- Simulates circular movement for each entity

### Mock Server

Monitors clients and sends segment commands:

```bash
# Basic usage
python3 mock_server.py

# Connect to remote broker
python3 mock_server.py --broker 192.168.1.100 --port 1883
```

**Arguments:**
- `--broker`: MQTT broker address (default: `localhost`)
- `--port`: MQTT broker port (default: `1883`)

**What it does:**
- Monitors all connected clients and their entities
- Prints status every 5 seconds
- Sends segment commands to all entities every 3 seconds
- Tracks heartbeat freshness

## Testing Scenarios

### 1. Basic End-to-End Test

```bash
# Terminal 1: Start broker
docker-compose up

# Terminal 2: Start mock server
python3 mock_server.py

# Terminal 3: Start mock client
python3 mock_client.py
```

You should see:
- Client connecting and subscribing
- Heartbeats being sent
- State reports being published
- Server monitoring output every 5s
- Segment commands being sent and received

### 2. Multiple Clients Test

```bash
# Terminal 1: Mock server
python3 mock_server.py

# Terminal 2: Client 1
python3 mock_client.py --client-id robot_01 --entities 1 2

# Terminal 3: Client 2
python3 mock_client.py --client-id robot_02 --entities 10 20 30

# Terminal 4: Client 3
python3 mock_client.py --client-id robot_03 --entities 100
```

### 3. Interoperability Test

Test Python mocks with Rust implementations:

```bash
# Python client with Rust server
cd mqtt-server && cargo run &
cd python_mocks && python3 mock_client.py

# Rust client with Python server
cd python_mocks && python3 mock_server.py &
cd rpi-mqtt-client && cargo run
```

### 4. Automated Testing

Run the complete test suite:

```bash
chmod +x test_system.sh
./test_system.sh
```

This will:
- Verify broker is running
- Test basic connectivity
- Test multiple clients
- Test Rust/Python interoperability (if binaries exist)

## Message Format Compatibility

The Python mocks use **struct** to pack/unpack binary data in the same format as Rust's bincode:

### Heartbeat
```python
struct.pack('<Q', timestamp)  # Little-endian u64
```

### State Report
```python
struct.pack('<Iiiiii',        # Little-endian
    entity_id,                 # u32
    x, y, angle,              # i32, i32, i32
    vx, vy                    # i32, i32
)
```

### Segment
```python
struct.pack('<IiiiiiiB',      # Little-endian
    entity_id,                 # u32
    start_x, start_y,         # i32, i32
    end_x, end_y,             # i32, i32
    vel_start, vel_end,       # i32, i32
    segment_type              # u8
)
```

## Output Examples

### Client Output
```
[mock_rpi_001] Starting mock client with 3 entities
[mock_rpi_001] Connected with result code 0
[mock_rpi_001] Subscribed to: server/mock_rpi_001/segment
[mock_rpi_001] Received segment for entity 1:
  Start: (0, 0)
  End: (1000, 1000)
  Velocity Start: 550
  Velocity End: 330
  Type: 0
```

### Server Output
```
[Server] Connected with result code 0
[Server] Subscribed to RPI topics
[Server] Sent segment to mock_rpi_001 for entity 1: type=0

=== Active Clients: 1 ===
  mock_rpi_001: last_hb=245ms ago, entities=3
    Entity 1: pos=(809, 587), angle=54000, vel=(110, 55)
    Entity 2: pos=(1309, -809), angle=234000, vel=(120, 60)
    Entity 3: pos=(1500, -951), angle=54000, vel=(130, 65)
```

## Performance Testing

Test high-frequency scenarios:

```python
# Modify mock_client.py to increase frequency
state_interval = 0.01  # 100 Hz instead of 10 Hz
heartbeat_interval = 0.1  # 10 Hz instead of 2 Hz
```

## Debugging

Enable verbose MQTT logging:

```python
# Add to mock_client.py or mock_server.py
import logging
logging.basicConfig(level=logging.DEBUG)
self.client.enable_logger()
```

## Troubleshooting

**"Connection refused"**
- Ensure MQTT broker is running: `docker ps | grep mqtt-broker`
- Check broker address and port

**"No module named 'paho'"**
- Install dependencies: `pip3 install -r requirements.txt`

**Messages not being received**
- Check topic names match between client/server
- Verify QoS settings
- Check broker logs: `docker-compose logs -f mosquitto`

**Binary format mismatch**
- Ensure struct pack/unpack formats match exactly
- Check endianness (should be little-endian `<`)
- Verify data types (u32 vs i32)

## Advanced Usage

### Custom Entity Behavior

Modify `mock_client.py` to implement custom movement patterns:

```python
def publish_state(self, entity_id: int, offset: int):
    # Custom behavior per entity
    if entity_id == 1:
        # Linear movement
        x = self.counter * 10
        y = 0
    elif entity_id == 2:
        # Circular movement
        rad = math.radians(self.counter)
        x = int(math.sin(rad) * 1000)
        y = int(math.cos(rad) * 1000)
    # ... etc
```

### Segment Processing

Add custom segment handlers:

```python
def on_message(self, client, userdata, msg):
    segment = deserialize_segment(msg.payload)
    
    # Route to entity-specific handler
    if segment.entity_id == 1:
        self.handle_entity1_segment(segment)
    elif segment.entity_id == 2:
        self.handle_entity2_segment(segment)
```

## CI/CD Integration

Use Python mocks for automated testing:

```yaml
# .github/workflows/test.yml
- name: Test MQTT System
  run: |
    docker-compose up -d
    pip3 install -r requirements.txt
    ./test_system.sh
```
