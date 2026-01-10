use anyhow::Result;
use rumqttc::{AsyncClient, MqttOptions, QoS, Event, Packet};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::sync::Arc;
use std::time::Duration;
use tokio::sync::Mutex;
use tokio::time;

const BROKER_HOST: &str = "localhost";
const BROKER_PORT: u16 = 1883;

#[derive(Serialize, Deserialize, Debug, Clone)]
struct Pose2D {
    x: i32,
    y: i32,
    angle: i32,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
struct Velocity2D {
    vx: i32,
    vy: i32,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
struct StateReport {
    entity_id: u32,
    pose: Pose2D,
    velocity: Velocity2D,
}

#[derive(Serialize, Deserialize, Debug)]
struct Heartbeat {
    timestamp: u64,
}

#[derive(Serialize, Deserialize, Debug)]
struct Segment {
    entity_id: u32,
    start: (i32, i32),
    end: (i32, i32),
    velocity_start: i32,
    velocity_end: i32,
    segment_type: u8,
}

#[derive(Debug, Clone)]
struct EntityState {
    last_state: StateReport,
    last_update: u64,
}

#[derive(Debug, Clone)]
struct ClientState {
    last_heartbeat: u64,
    entities: HashMap<u32, EntityState>, // entity_id -> EntityState
}

type ClientMap = Arc<Mutex<HashMap<String, ClientState>>>;

async fn send_segment(client: &AsyncClient, client_id: &str, segment: Segment) -> Result<()> {
    let payload = bincode::serialize(&segment)?;
    client.publish(
        format!("server/{}/segment", client_id),
        QoS::AtLeastOnce,
        false,
        payload,
    ).await?;
    
    println!("Sent segment to {} for entity {}: type={}", 
        client_id, segment.entity_id, segment.segment_type);
    Ok(())
}

fn extract_client_id(topic: &str) -> Option<String> {
    // Topics: "rpi/{client_id}/heartbeat" or "rpi/{client_id}/state"
    let parts: Vec<&str> = topic.split('/').collect();
    if parts.len() >= 3 && parts[0] == "rpi" {
        Some(parts[1].to_string())
    } else {
        None
    }
}

#[tokio::main]
async fn main() -> Result<()> {
    println!("Starting MQTT Server");
    
    let mut mqttoptions = MqttOptions::new("mqtt_server", BROKER_HOST, BROKER_PORT);
    mqttoptions.set_keep_alive(Duration::from_secs(5));
    
    let (client, mut eventloop) = AsyncClient::new(mqttoptions, 10);
    
    // Subscribe to all RPI topics
    client.subscribe("rpi/+/heartbeat", QoS::AtLeastOnce).await?;
    client.subscribe("rpi/+/state", QoS::AtLeastOnce).await?;
    
    println!("Subscribed to RPI topics");
    
    let clients: ClientMap = Arc::new(Mutex::new(HashMap::new()));
    
    // Spawn monitoring task
    let clients_monitor = clients.clone();
    tokio::spawn(async move {
        let mut interval = time::interval(Duration::from_secs(5));
        loop {
            interval.tick().await;
            let clients = clients_monitor.lock().await;
            println!("\n=== Active Clients: {} ===", clients.len());
            for (id, state) in clients.iter() {
                println!("  {}: last_hb={}ms ago, entities={}", 
                    id,
                    std::time::SystemTime::now()
                        .duration_since(std::time::UNIX_EPOCH)
                        .unwrap()
                        .as_millis() as u64 - state.last_heartbeat,
                    state.entities.len()
                );
                for (entity_id, entity) in state.entities.iter() {
                    println!("    Entity {}: pos=({}, {}), angle={}", 
                        entity_id,
                        entity.last_state.pose.x, 
                        entity.last_state.pose.y, 
                        entity.last_state.pose.angle
                    );
                }
            }
        }
    });
    
    // Spawn segment sending task
    let client_sender = client.clone();
    let clients_sender = clients.clone();
    tokio::spawn(async move {
        let mut interval = time::interval(Duration::from_secs(3));
        let mut seg_type = 0u8;
        
        loop {
            interval.tick().await;
            
            let clients = clients_sender.lock().await;
            
            // Send segments to all entities of all clients
            for (client_id, client_state) in clients.iter() {
                for &entity_id in client_state.entities.keys() {
                    let segment = Segment {
                        entity_id,
                        start: (0, 0),
                        end: (1000 * entity_id as i32, 1000),
                        velocity_start: 500 + (entity_id as i32 * 50),
                        velocity_end: 300 + (entity_id as i32 * 30),
                        segment_type: seg_type,
                    };
                    
                    if let Err(e) = send_segment(&client_sender, client_id, segment).await {
                        eprintln!("Failed to send segment: {}", e);
                    }
                }
            }
            
            seg_type = seg_type.wrapping_add(1);
        }
    });
    
    // Event loop
    loop {
        match eventloop.poll().await {
            Ok(Event::Incoming(Packet::Publish(p))) => {
                if let Some(client_id) = extract_client_id(&p.topic) {
                    let mut clients = clients.lock().await;
                    
                    if p.topic.ends_with("/heartbeat") {
                        if let Ok(hb) = bincode::deserialize::<Heartbeat>(&p.payload) {
                            clients.entry(client_id.clone())
                                .or_insert_with(|| ClientState {
                                    last_heartbeat: 0,
                                    entities: HashMap::new(),
                                })
                                .last_heartbeat = hb.timestamp;
                        }
                    } else if p.topic.ends_with("/state") {
                        if let Ok(state) = bincode::deserialize::<StateReport>(&p.payload) {
                            let entity_id = state.entity_id;
                            let now = std::time::SystemTime::now()
                                .duration_since(std::time::UNIX_EPOCH)
                                .unwrap()
                                .as_millis() as u64;
                            
                            let client_entry = clients.entry(client_id.clone())
                                .or_insert_with(|| ClientState {
                                    last_heartbeat: 0,
                                    entities: HashMap::new(),
                                });
                            
                            client_entry.entities.insert(entity_id, EntityState {
                                last_state: state,
                                last_update: now,
                            });
                        }
                    }
                }
            }
            Ok(_) => {}
            Err(e) => {
                eprintln!("Event loop error: {}", e);
                time::sleep(Duration::from_secs(1)).await;
            }
        }
    }
}
