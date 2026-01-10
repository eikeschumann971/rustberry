#!/usr/bin/env python3
"""
Mock MQTT Server for testing
Monitors client heartbeats and states, sends segment commands
"""

import paho.mqtt.client as mqtt
import time
import struct
import argparse
from dataclasses import dataclass
from collections import defaultdict
from typing import Dict, Optional


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
class EntityState:
    last_state: StateReport
    last_update: float


@dataclass
class ClientState:
    last_heartbeat: float
    entities: Dict[int, EntityState]


def deserialize_heartbeat(data: bytes) -> int:
    """Deserialize heartbeat from Rust bincode format"""
    return struct.unpack('<Q', data)[0]


def deserialize_state(data: bytes) -> StateReport:
    """Deserialize state report from Rust bincode format"""
    # u32 entity_id, i32 x, i32 y, i32 angle, i32 vx, i32 vy
    unpacked = struct.unpack('<Iiiiii', data)
    return StateReport(
        entity_id=unpacked[0],
        pose=Pose2D(x=unpacked[1], y=unpacked[2], angle=unpacked[3]),
        velocity=Velocity2D(vx=unpacked[4], vy=unpacked[5])
    )


def serialize_segment(entity_id: int, start: tuple, end: tuple, 
                      vel_start: int, vel_end: int, seg_type: int) -> bytes:
    """Serialize segment to match Rust bincode format"""
    # u32 entity_id, i32 start_x, i32 start_y, i32 end_x, i32 end_y,
    # i32 vel_start, i32 vel_end, u8 type
    return struct.pack(
        '<IiiiiiiB',
        entity_id,
        start[0], start[1],
        end[0], end[1],
        vel_start,
        vel_end,
        seg_type
    )


class MockServer:
    def __init__(self, broker: str, port: int):
        self.broker = broker
        self.port = port
        self.clients: Dict[str, ClientState] = {}
        self.segment_counter = 0
        
        self.client = mqtt.Client(client_id="mock_server")
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        
    def on_connect(self, client, userdata, flags, rc):
        print(f"[Server] Connected with result code {rc}")
        # Subscribe to all RPI topics
        client.subscribe("rpi/+/heartbeat")
        client.subscribe("rpi/+/state")
        print("[Server] Subscribed to RPI topics")
        
    def on_message(self, client, userdata, msg):
        topic_parts = msg.topic.split('/')
        
        if len(topic_parts) >= 3 and topic_parts[0] == 'rpi':
            client_id = topic_parts[1]
            msg_type = topic_parts[2]
            
            if msg_type == 'heartbeat':
                self.handle_heartbeat(client_id, msg.payload)
            elif msg_type == 'state':
                self.handle_state(client_id, msg.payload)
    
    def handle_heartbeat(self, client_id: str, payload: bytes):
        """Handle incoming heartbeat"""
        try:
            timestamp = deserialize_heartbeat(payload)
            now = time.time()
            
            if client_id not in self.clients:
                self.clients[client_id] = ClientState(
                    last_heartbeat=now,
                    entities={}
                )
            else:
                self.clients[client_id].last_heartbeat = now
                
        except Exception as e:
            print(f"[Server] Error deserializing heartbeat from {client_id}: {e}")
    
    def handle_state(self, client_id: str, payload: bytes):
        """Handle incoming state report"""
        try:
            state = deserialize_state(payload)
            now = time.time()
            
            if client_id not in self.clients:
                self.clients[client_id] = ClientState(
                    last_heartbeat=0,
                    entities={}
                )
            
            self.clients[client_id].entities[state.entity_id] = EntityState(
                last_state=state,
                last_update=now
            )
            
        except Exception as e:
            print(f"[Server] Error deserializing state from {client_id}: {e}")
    
    def send_segment(self, client_id: str, entity_id: int, seg_type: int):
        """Send segment command to specific entity"""
        segment_data = serialize_segment(
            entity_id=entity_id,
            start=(0, 0),
            end=(1000 * entity_id, 1000),
            vel_start=500 + (entity_id * 50),
            vel_end=300 + (entity_id * 30),
            seg_type=seg_type
        )
        
        topic = f"server/{client_id}/segment"
        self.client.publish(topic, segment_data, qos=1)
        print(f"[Server] Sent segment to {client_id} for entity {entity_id}: type={seg_type}")
    
    def monitor_clients(self):
        """Print client status"""
        print(f"\n=== Active Clients: {len(self.clients)} ===")
        now = time.time()
        
        for client_id, state in self.clients.items():
            hb_age = (now - state.last_heartbeat) * 1000  # ms
            print(f"  {client_id}: last_hb={hb_age:.0f}ms ago, entities={len(state.entities)}")
            
            for entity_id, entity in state.entities.items():
                s = entity.last_state
                print(f"    Entity {entity_id}: pos=({s.pose.x}, {s.pose.y}), "
                      f"angle={s.pose.angle}, vel=({s.velocity.vx}, {s.velocity.vy})")
    
    def send_segments_to_all(self):
        """Send segments to all known entities"""
        for client_id, state in self.clients.items():
            for entity_id in state.entities.keys():
                self.send_segment(client_id, entity_id, self.segment_counter % 256)
        
        self.segment_counter += 1
    
    def run(self):
        """Main loop"""
        print("[Server] Starting mock server")
        self.client.connect(self.broker, self.port, 60)
        self.client.loop_start()
        
        monitor_interval = 5.0  # 5 seconds
        segment_interval = 3.0  # 3 seconds
        
        last_monitor = time.time()
        last_segment = time.time()
        
        try:
            while True:
                now = time.time()
                
                # Monitor clients every 5 seconds
                if now - last_monitor >= monitor_interval:
                    self.monitor_clients()
                    last_monitor = now
                
                # Send segments every 3 seconds
                if now - last_segment >= segment_interval:
                    if self.clients:  # Only if we have clients
                        self.send_segments_to_all()
                    last_segment = now
                
                time.sleep(0.1)
                
        except KeyboardInterrupt:
            print("\n[Server] Shutting down...")
            self.client.loop_stop()
            self.client.disconnect()


def main():
    parser = argparse.ArgumentParser(description='Mock MQTT Server')
    parser.add_argument('--broker', default='localhost',
                       help='MQTT broker address (default: localhost)')
    parser.add_argument('--port', type=int, default=1883,
                       help='MQTT broker port (default: 1883)')
    
    args = parser.parse_args()
    
    server = MockServer(args.broker, args.port)
    server.run()


if __name__ == '__main__':
    main()
    