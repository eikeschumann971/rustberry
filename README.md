# MQTT RaspberryPi Fleet Communication System

A complete MQTT-based communication system for managing a fleet of Raspberry Pi devices with a central server.

## Origin
  - https://claude.ai/chat/cda74945-7244-4719-944c-0c4acf683741
  - https://m365.cloud.microsoft/chat/?auth=1&origindomain=microsoft365&client-request-id=1016845e-9396-4907-9a5e-e23d633122ad

## System Architecture

- **MQTT Broker**: Eclipse Mosquitto (runs in Docker)
- **Raspberry Pi Client**: Rust application sending heartbeats (500ms) and state reports
- **Server**: Rust application monitoring clients and sending segment commands

## Message Formats

### From Raspberry Pi to Server:
- **Heartbeat** (every 500ms): `{timestamp: u64}` - one per client
- **State Report** (configurable rate): `{entity_id: u32, pose: {x, y, angle}, velocity: {vx, vy}}`
  - All coordinates are 32-bit integers
  - Each Raspberry Pi can manage multiple entities (vehicles)

### From Server to Raspberry Pi:
- **Segment**: `{entity_id: u32, start: (i32, i32), end: (i32, i32), velocity_start: i32, velocity_end: i32, segment_type: u8}`
  - Targeted to specific entity on the client

## Setup Instructions

### 1. Start MQTT Broker

```bash
# Create necessary directories
mkdir -p mosquitto/config mosquitto/data mosquitto/log

# Copy mosquitto.conf to mosquitto/config/
cp mosquitto.conf mosquitto/config/

# Start the broker
docker-compose up -d

# Check broker status
docker-compose logs -f mosquitto
```

### 2. Build and Run Server

```bash
cd mqtt-server
cargo build --release
cargo run --release
```

The server will:
- Subscribe to all RPI heartbeats and state reports
- Monitor connected clients and their entities
- Send segment commands to specific entities every 3 seconds

### 3. Build and Run Raspberry Pi Client

```bash
cd rpi-mqtt-client
cargo build --release
cargo run --release
```

**For multiple Raspberry Pi devices**, change the `CLIENT_ID` constant in `main.rs`:
```rust
const CLIENT_ID: &str = "rpi_client_002"; // Unique ID per device
```

**For multiple entities per client**, modify the `entity_ids` vector:
```rust
let entity_ids = vec![1u32, 2u32, 3u32]; // Add more entity IDs as needed
```

### 4. Testing Without Raspberry Pi

You can run the client on your development machine to test:
```bash
# Terminal 1: Start broker
docker-compose up

# Terminal 2: Start server
cd mqtt-server && cargo run

# Terminal 3: Start client 1
cd rpi-mqtt-client && cargo run

# Terminal 4: Start client 2 (modify CLIENT_ID first)
cd rpi-mqtt-client && cargo run
```

## MQTT Topics

- `rpi/{client_id}/heartbeat` - Client heartbeat messages
- `rpi/{client_id}/state` - Client state reports
- `server/{client_id}/segment` - Server segment commands

## Configuration

### Broker Connection
Both client and server connect to `localhost:1883` by default. For production:

**Client (`rpi-mqtt-client/src/main.rs`):**
```rust
const BROKER_HOST: &str = "your-server-ip";
```

**Server (`mqtt-server/src/main.rs`):**
```rust
const BROKER_HOST: &str = "localhost"; // Usually stays localhost
```

### Message Rates

**Client heartbeat rate:**
```rust
let mut interval = time::interval(Duration::from_millis(500));
```

**Client state report rate:**
```rust
let mut interval = time::interval(Duration::from_millis(100)); // 10 Hz
```

**Server segment sending rate:**
```rust
let mut interval = time::interval(Duration::from_secs(3));
```

## Production Deployment

### Security Enhancements

1. **Enable Authentication** in `mosquitto.conf`:
```conf
allow_anonymous false
password_file /mosquitto/config/passwd
```

Generate password file:
```bash
docker exec -it mqtt-broker mosquitto_passwd -c /mosquitto/config/passwd username
```

2. **Enable TLS/SSL**:
```conf
listener 8883
cafile /mosquitto/config/ca.crt
certfile /mosquitto/config/server.crt
keyfile /mosquitto/config/server.key
```

3. **Use QoS 1 or 2** for critical messages (already configured in code)

### Performance Tuning

For large fleets (100+ devices):
- Increase `max_connections` in mosquitto.conf
- Adjust `max_queued_messages` based on message rates
- Consider running broker on dedicated hardware
- Monitor broker metrics

### Monitoring

Check broker statistics:
```bash
# Subscribe to system topics
mosquitto_sub -h localhost -t '$SYS/#' -v
```

Monitor specific metrics:
- `$SYS/broker/clients/connected` - Connected clients
- `$SYS/broker/messages/received` - Message throughput
- `$SYS/broker/load/messages/sent/1min` - Send rate

## Message Size Analysis

- **Heartbeat**: ~16 bytes (timestamp + overhead)
- **State Report**: ~28 bytes (entity_id + 6 × i32 + overhead)
- **Segment**: ~25 bytes (entity_id + 6 × i32 + 1 × u8 + overhead)

All well within your 16-64 byte requirement!

## Multi-Entity Architecture

Each Raspberry Pi client can manage multiple entities (vehicles, robots, etc.):

**Client Side:**
- One heartbeat per client (500ms)
- State reports for each entity (configurable rate)
- Receives segments tagged with `entity_id`
- Routes segments to appropriate entity handler

**Server Side:**
- Tracks all clients and their entities
- Sends segments targeted to specific entities
- Monitors entity states independently

Example output:
```
=== Active Clients: 1 ===
  rpi_client_001: last_hb=127ms ago, entities=3
    Entity 1: pos=(123, 456), angle=45000
    Entity 2: pos=(789, 012), angle=90000
    Entity 3: pos=(345, 678), angle=135000
```

## Troubleshooting

**Connection refused:**
- Check broker is running: `docker-compose ps`
- Verify port 1883 is accessible: `netstat -an | grep 1883`

**Messages not received:**
- Check topic subscriptions match publish topics
- Verify client IDs are unique
- Check QoS levels

**High latency:**
- Check network connectivity
- Reduce message rates
- Increase broker resources
