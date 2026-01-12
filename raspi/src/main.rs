use anyhow::Result;
use rumqttc::{AsyncClient, MqttOptions, QoS, Event, Packet};
use serde::{Deserialize, Serialize};
use std::time::Duration;
use tokio::time;

const BROKER_HOST: &str = "localhost";
const BROKER_PORT: u16 = 1883;
const CLIENT_ID: &str = "rpi_client_001"; // Change for each Pi

#[derive(Serialize, Deserialize, Debug)]
struct Pose2D {
    x: i32,
    y: i32,
    angle: i32, // In millidegrees (e.g., 90000 = 90 degrees)
}

#[derive(Serialize, Deserialize, Debug)]
struct Velocity2D {
    vx: i32,
    vy: i32,
}

#[derive(Serialize, Deserialize, Debug)]
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

async fn publish_heartbeat(client: &AsyncClient) -> Result<()> {
    let heartbeat = Heartbeat {
        timestamp: std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)?
            .as_millis() as u64,
    };
    
    let payload = bincode::serialize(&heartbeat)?;
    client.publish(
        format!("rpi/{}/heartbeat", CLIENT_ID),
        QoS::AtLeastOnce,
        false,
        payload,
    ).await?;
    
    Ok(())
}

async fn publish_state(client: &AsyncClient, state: &StateReport) -> Result<()> {
    let payload = bincode::serialize(state)?;
    client.publish(
        format!("rpi/{}/state", CLIENT_ID),
        QoS::AtLeastOnce,
        false,
        payload,
    ).await?;
    
    Ok(())
}

fn handle_segment(segment: Segment) {
    println!("Received segment for entity {}:", segment.entity_id);
    println!("  Start: ({}, {})", segment.start.0, segment.start.1);
    println!("  End: ({}, {})", segment.end.0, segment.end.1);
    println!("  Velocity Start: {}", segment.velocity_start);
    println!("  Velocity End: {}", segment.velocity_end);
    println!("  Type: {}", segment.segment_type);
    
    // TODO: Route to appropriate entity/vehicle handler
    // match segment.entity_id {
    //     1 => vehicle1.process_segment(segment),
    //     2 => vehicle2.process_segment(segment),
    //     _ => eprintln!("Unknown entity_id: {}", segment.entity_id),
    // }
}

#[tokio::main]
async fn main() -> Result<()> {
    println!("Starting Raspberry Pi MQTT Client: {}", CLIENT_ID);
    
    println!("Running BLAS test...");
    blas::run_blas_example();

    println!("Running ODE solver...");
    ode::run_ode_example();

    let mut mqttoptions = MqttOptions::new(CLIENT_ID, BROKER_HOST, BROKER_PORT);
    mqttoptions.set_keep_alive(Duration::from_secs(5));
    
    let (client, mut eventloop) = AsyncClient::new(mqttoptions, 10);
    
    // Subscribe to segment commands
    client.subscribe(
        format!("server/{}/segment", CLIENT_ID),
        QoS::AtLeastOnce,
    ).await?;
    
    println!("Subscribed to: server/{}/segment", CLIENT_ID);
    
    // Spawn heartbeat task
    let client_hb = client.clone();
    tokio::spawn(async move {
        let mut interval = time::interval(Duration::from_millis(500));
        loop {
            interval.tick().await;
            if let Err(e) = publish_heartbeat(&client_hb).await {
                eprintln!("Heartbeat error: {}", e);
            }
        }
    });
    
    // Spawn state reporting task for multiple entities
    let client_state = client.clone();
    tokio::spawn(async move {
        let mut interval = time::interval(Duration::from_millis(100));
        let mut counter = 0i32;
        
        // Simulate managing 3 entities/vehicles
        let entity_ids = vec![1u32, 2u32, 3u32];
        
        loop {
            interval.tick().await;
            
            // Send state for each entity
            for (idx, &entity_id) in entity_ids.iter().enumerate() {
                let offset = idx as i32 * 120; // Different phase for each entity
                let angle = ((counter + offset) % 360) * 1000; // millidegrees
                let rad = ((counter + offset) as f64 * 0.01745).sin();
                
                let state = StateReport {
                    entity_id,
                    pose: Pose2D {
                        x: (rad * 1000.0) as i32 + (idx as i32 * 500),
                        y: (((counter + offset) as f64 * 0.01745).cos() * 1000.0) as i32,
                        angle,
                    },
                    velocity: Velocity2D {
                        vx: 100 + (idx as i32 * 10),
                        vy: 50 + (idx as i32 * 5),
                    },
                };
                
                if let Err(e) = publish_state(&client_state, &state).await {
                    eprintln!("State publish error for entity {}: {}", entity_id, e);
                }
            }
            
            counter += 1;
        }
    });
    
    // Event loop for receiving messages
    loop {
        match eventloop.poll().await {
            Ok(Event::Incoming(Packet::Publish(p))) => {
                if let Ok(segment) = bincode::deserialize::<Segment>(&p.payload) {
                    handle_segment(segment);
                } else {
                    eprintln!("Failed to deserialize segment");
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
