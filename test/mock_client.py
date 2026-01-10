#!/usr/bin/env python3
"""
Mock Raspberry Pi MQTT Client for testing
Sends heartbeats and state reports for multiple entities
"""

import paho.mqtt.client as mqtt
import msgpack
import time
import math
import struct
import argparse
from dataclasses import dataclass
from typing import List


@dataclass
class Pose2D:
    x: int
    y: int
    angle: int


@dataclass
class Velocity2D:
    vx: int
    vy: int


@dataclass
class StateReport:
    entity_id: int
    pose: Pose2D
    velocity: Velocity2D


@dataclass
class Segment:
    entity_id: int
    start: tuple[int, int]
    end: tuple[int, int]
    velocity_start: int
    velocity_end: int
    segment_type: int


def serialize_heartbeat(timestamp: int) -> bytes:
    """Serialize heartbeat to match Rust bincode format"""
    return struct.pack('<Q', timestamp)


def serialize_state(state: StateReport) -> bytes:
    """Serialize state report to match Rust bincode format"""
    # bincode serializes in little-endian
    # u32 entity_id, i32 x, i32 y, i32 angle, i32 vx, i32 vy
    return struct.pack(
        '<Iiiiii',
        state.entity_id,
        state.pose.x,
        state.pose.y,
        state.pose.angle,
        state.velocity.vx,
        state.velocity.vy
    )


def deserialize_segment(data: bytes) -> Segment:
    """Deserialize segment from Rust bincode format"""
    # u32 entity_id, i32 start_x, i32 start_y, i32 end_x, i32 end_y, 
    # i32 vel_start, i32 vel_end, u8 type
    unpacked = struct.unpack('<IiiiiiiB', data)
    return Segment(
        entity_id=unpacked[0],
        start=(unpacked[1], unpacked[2]),
        end=(unpacked[3], unpacked[4]),
        velocity_start=unpacked[5],
        velocity_end=unpacked[6],
        segment_type=unpacked[7]
    )


class MockClient:
    def __init__(self, client_id: str, broker: str, port: int, entity_ids: List[int]):
        self.client_id = client_id
        self.broker = broker
        self.port = port
        self.entity_ids = entity_ids
        self.counter = 0
        
        self.client = mqtt.Client(client_id=client_id)
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        
    def on_connect(self, client, userdata, flags, rc):
        print(f"[{self.client_id}] Connected with result code {rc}")
        # Subscribe to segment commands
        topic = f"server/{self.client_id}/segment"
        client.subscribe(topic)
        print(f"[{self.client_id}] Subscribed to: {topic}")
        
    def on_message(self, client, userdata, msg):
        try:
            segment = deserialize_segment(msg.payload)
            print(f"[{self.client_id}] Received segment for entity {segment.entity_id}:")
            print(f"  Start: {segment.start}")
            print(f"  End: {segment.end}")
            print(f"  Velocity Start: {segment.velocity_start}")
            print(f"  Velocity End: {segment.velocity_end}")
            print(f"  Type: {segment.segment_type}")
        except Exception as e:
            print(f"[{self.client_id}] Error deserializing segment: {e}")
    
    def publish_heartbeat(self):
        """Publish heartbeat message"""
        timestamp = int(time.time() * 1000)  # milliseconds
        payload = serialize_heartbeat(timestamp)
        topic = f"rpi/{self.client_id}/heartbeat"
        self.client.publish(topic, payload, qos=1)
        
    def publish_state(self, entity_id: int, offset: int):
        """Publish state for a specific entity"""
        # Simulate movement in a circle
        angle = ((self.counter + offset) % 360) * 1000  # millidegrees
        rad = math.radians((self.counter + offset) % 360)
        
        state = StateReport(
            entity_id=entity_id,
            pose=Pose2D(
                x=int(math.sin(rad) * 1000) + (entity_id * 500),
                y=int(math.cos(rad) * 1000),
                angle=angle
            ),
            velocity=Velocity2D(
                vx=100 + (entity_id * 10),
                vy=50 + (entity_id * 5)
            )
        )
        
        payload = serialize_state(state)
        topic = f"rpi/{self.client_id}/state"
        self.client.publish(topic, payload, qos=1)
        
    def run(self):
        """Main loop"""
        print(f"[{self.client_id}] Starting mock client with {len(self.entity_ids)} entities")
        self.client.connect(self.broker, self.port, 60)
        self.client.loop_start()
        
        heartbeat_interval = 0.5  # 500ms
        state_interval = 0.1      # 100ms (10 Hz)
        
        last_heartbeat = time.time()
        last_state = time.time()
        
        try:
            while True:
                now = time.time()
                
                # Send heartbeat every 500ms
                if now - last_heartbeat >= heartbeat_interval:
                    self.publish_heartbeat()
                    last_heartbeat = now
                
                # Send state for all entities every 100ms
                if now - last_state >= state_interval:
                    for idx, entity_id in enumerate(self.entity_ids):
                        offset = idx * 120  # Different phase per entity
                        self.publish_state(entity_id, offset)
                    last_state = now
                    self.counter += 1
                
                time.sleep(0.01)  # Small sleep to prevent busy loop
                
        except KeyboardInterrupt:
            print(f"\n[{self.client_id}] Shutting down...")
            self.client.loop_stop()
            self.client.disconnect()


def main():
    parser = argparse.ArgumentParser(description='Mock Raspberry Pi MQTT Client')
    parser.add_argument('--client-id', default='mock_rpi_001', 
                       help='Client ID (default: mock_rpi_001)')
    parser.add_argument('--broker', default='localhost',
                       help='MQTT broker address (default: localhost)')
    parser.add_argument('--port', type=int, default=1883,
                       help='MQTT broker port (default: 1883)')
    parser.add_argument('--entities', nargs='+', type=int, default=[1, 2, 3],
                       help='Entity IDs to simulate (default: 1 2 3)')
    
    args = parser.parse_args()
    
    client = MockClient(args.client_id, args.broker, args.port, args.entities)
    client.run()


if __name__ == '__main__':
    main()