# Entity-Component-System (ECS) Architecture

ECS is a software architectural pattern commonly used in game development and simulations that emphasizes composition over inheritance. It separates data from behavior and organizes code into three main concepts:

## Core Concepts

**Entities** are unique identifiers (often just an ID) that represent objects in your system. They don't contain data or logic themselves - they're just containers that tie components together.

**Components** are plain data structures with no behavior. Each component represents one aspect of an entity (like position, velocity, or mesh data). An entity is defined by which components it has.

**Systems** contain the logic and operate on entities that have specific component combinations. Systems process data and implement behavior.

## ECS for Vehicle Simulation

Yes, ECS is excellent for car-like vehicle simulations in C++! Here's why:

**Performance Benefits**: ECS enables cache-friendly data layouts. Since components of the same type are stored contiguously in memory, systems can process them very efficiently - crucial for simulations with many vehicles.

**Flexibility**: You can easily create different vehicle types by mixing components. A basic car might have Position, Velocity, and SteeringComponents, while a race car adds AerodynamicsComponent and TireWearComponent.

**Scalability**: Adding new features (like damage systems, AI controllers, or sensor data) doesn't require modifying existing vehicle classes - you just add new components and systems.

## Example Structure for Vehicle Simulation

For a driving simulation, you might have:

**Components**: TransformComponent (position, rotation), PhysicsComponent (velocity, acceleration, mass), WheelComponent (tire friction, suspension), EngineComponent (power, RPM), SteeringComponent (angle, max turn rate), SensorComponent (for autonomous vehicles)

**Systems**: PhysicsSystem (applies forces, handles collisions), SteeringSystem (processes input, updates wheel angles), RenderSystem (draws vehicles), AISystem (autonomous driving logic)

Popular C++ ECS libraries like EnTT make implementation straightforward and performant. The pattern really shines when you have multiple vehicles with varying characteristics or need to simulate complex interactions between many entities.

# Using EnTT with Vehicle Fleets in SoA Fashion

EnTT naturally stores component data in a Structure of Arrays (SoA) layout, which is perfect for high-performance vehicle simulations. Here's how to leverage this for a fleet:

## Basic Setup

```cpp
#include <entt/entt.hpp>
#include <glm/glm.hpp> // or your preferred math library

// Components - keep them simple, data-only
struct Transform {
    glm::vec3 position;
    glm::quat rotation;
};

struct Physics {
    glm::vec3 velocity;
    glm::vec3 acceleration;
    float mass;
};

struct Wheel {
    float steeringAngle;
    float friction;
    float suspensionCompression;
};

struct Engine {
    float rpm;
    float maxPower;
    float throttle;
};

// Create registry (manages all entities and components)
entt::registry registry;
```

## Creating Vehicle Fleet

```cpp
// Create multiple vehicles efficiently
std::vector<entt::entity> createVehicleFleet(entt::registry& reg, int count) {
    std::vector<entt::entity> vehicles;
    vehicles.reserve(count);
    
    for (int i = 0; i < count; ++i) {
        auto vehicle = reg.create();
        
        // Add components - EnTT stores these in separate arrays
        reg.emplace<Transform>(vehicle, 
            glm::vec3(i * 5.0f, 0.0f, 0.0f), // position
            glm::quat(1, 0, 0, 0)             // rotation
        );
        
        reg.emplace<Physics>(vehicle,
            glm::vec3(0.0f),  // velocity
            glm::vec3(0.0f),  // acceleration
            1500.0f           // mass (kg)
        );
        
        reg.emplace<Engine>(vehicle, 0.0f, 150.0f, 0.0f);
        
        vehicles.push_back(vehicle);
    }
    
    return vehicles;
}
```

## Processing with SoA Benefits

EnTT's views let you iterate efficiently over component arrays:

```cpp
// Physics update system - processes all vehicles efficiently
void updatePhysics(entt::registry& reg, float deltaTime) {
    // View creates a SoA-friendly iteration
    auto view = reg.view<Transform, Physics>();
    
    // This loop is cache-friendly - components are stored contiguously
    for (auto entity : view) {
        auto& transform = view.get<Transform>(entity);
        auto& physics = view.get<Physics>(entity);
        
        // Update velocity
        physics.velocity += physics.acceleration * deltaTime;
        
        // Update position
        transform.position += physics.velocity * deltaTime;
        
        // Apply drag
        physics.velocity *= 0.99f;
    }
}

// Engine system
void updateEngines(entt::registry& reg, float deltaTime) {
    auto view = reg.view<Engine, Physics>();
    
    for (auto entity : view) {
        auto& engine = view.get<Engine>(entity);
        auto& physics = view.get<Physics>(entity);
        
        // Calculate force from engine
        float force = engine.throttle * engine.maxPower;
        
        // Apply to acceleration (F = ma, so a = F/m)
        physics.acceleration.x = force / physics.mass;
        
        // Update RPM based on throttle
        engine.rpm = engine.throttle * 6000.0f;
    }
}
```

## Efficient Multi-Component Access

```cpp
// Get raw component arrays for maximum performance
void optimizedPhysicsUpdate(entt::registry& reg, float deltaTime) {
    // Get storage for direct access
    auto& transforms = reg.storage<Transform>();
    auto& physicsComps = reg.storage<Physics>();
    
    // This is extremely cache-friendly iteration
    for (size_t i = 0; i < transforms.size(); ++i) {
        auto& transform = transforms[i];
        auto& physics = physicsComps[i];
        
        physics.velocity += physics.acceleration * deltaTime;
        transform.position += physics.velocity * deltaTime;
    }
}
```

## Advanced: Groups for Even Better Performance

For frequently accessed component combinations, use groups:

```cpp
// Create a group - EnTT optimizes storage layout for these components
void setupOptimizedAccess(entt::registry& reg) {
    // This ensures Transform and Physics are stored optimally together
    reg.group<Transform>(entt::get<Physics>);
}

void updateWithGroup(entt::registry& reg, float deltaTime) {
    // Groups provide the fastest possible iteration
    auto group = reg.group<Transform>(entt::get<Physics>);
    
    for (auto entity : group) {
        auto [transform, physics] = group.get<Transform, Physics>(entity);
        
        physics.velocity += physics.acceleration * deltaTime;
        transform.position += physics.velocity * deltaTime;
    }
}
```

## Complete Update Loop

```cpp
void simulationStep(entt::registry& reg, float deltaTime) {
    // Systems process components in sequence
    updateEngines(reg, deltaTime);
    updatePhysics(reg, deltaTime);
    // updateCollisions(reg, deltaTime);
    // updateSteering(reg, deltaTime);
    // updateRendering(reg);
}

int main() {
    entt::registry registry;
    
    // Create fleet of 1000 vehicles
    auto fleet = createVehicleFleet(registry, 1000);
    
    // Simulation loop
    float deltaTime = 1.0f / 60.0f;
    while (true) {
        simulationStep(registry, deltaTime);
    }
}
```

The key advantages of EnTT's SoA approach for vehicle fleets:

- **Cache efficiency**: Components of the same type are stored contiguously, minimizing cache misses
- **SIMD potential**: Contiguous data enables vectorization optimizations
- **Flexibility**: Easy to add/remove components without restructuring code
- **Scalability**: Handles thousands of vehicles efficiently

EnTT automatically manages the SoA layout - you just focus on defining components and systems!

# GLM vs Eigen for Vehicle Simulation

Both are excellent, but I'd recommend **Eigen** for vehicle simulation work. Here's why:

## Eigen - Better for Simulations

**Performance**: Eigen is heavily optimized for numerical computations with excellent SIMD support. It's used in robotics, physics engines, and scientific computing where performance matters.

**Matrix operations**: Vehicle dynamics involve lots of matrix math (inertia tensors, coordinate transformations, Jacobians for constraints). Eigen excels at this with efficient linear algebra routines.

**Solver support**: Built-in solvers for linear systems, eigenvalue problems, and decompositions - useful for advanced physics like suspension systems or constraint-based vehicle dynamics.

**Fixed and dynamic sizes**: You can choose compile-time or runtime sizing based on your needs, with compile-time sizes giving zero-overhead abstraction.

```cpp
// Eigen for vehicle physics
Eigen::Vector3f velocity;
Eigen::Matrix3f inertiaTensor;
Eigen::Quaternionf rotation;

// Efficient matrix operations
Eigen::Vector3f angularMomentum = inertiaTensor * angularVelocity;
```

## GLM - Better for Graphics

**OpenGL compatibility**: GLM is designed to match GLSL types exactly. If you're doing heavy rendering work, GLM's API feels natural.

**Simpler API**: More straightforward for basic 3D graphics math without the learning curve of Eigen's expression templates.

**Lighter weight**: Smaller library, faster compile times, less template complexity.

```cpp
// GLM for graphics
glm::vec3 position;
glm::mat4 modelMatrix;
glm::quat rotation;

// Direct compatibility with OpenGL
glUniformMatrix4fv(location, 1, GL_FALSE, glm::value_ptr(modelMatrix));
```

## My Recommendation for Vehicle Simulation

**Use Eigen** because:

1. **Physics-first**: Vehicle simulation is fundamentally about physics, not graphics. Eigen's numerical robustness and performance are crucial for stable integration and constraint solving.

2. **Scalability**: As your simulation grows (adding tire models, aerodynamics, suspension kinematics), you'll need Eigen's advanced features.

3. **Industry standard**: Physics engines (Bullet, PhysX internals), robotics frameworks (ROS), and simulation tools use Eigen.

4. **Better numerics**: More stable algorithms for operations that matter in simulation (matrix inversions, solving linear systems for constraints).

## Practical Compromise

If you're building a simulation with significant graphics components, you can use both:

```cpp
// Use Eigen for physics
struct Physics {
    Eigen::Vector3f velocity;
    Eigen::Vector3f acceleration;
    Eigen::Matrix3f inertiaTensor;
};

// Use GLM for rendering
struct RenderTransform {
    glm::vec3 position;
    glm::quat rotation;
    glm::mat4 modelMatrix;
};

// Convert when needed
glm::vec3 toGLM(const Eigen::Vector3f& v) {
    return glm::vec3(v.x(), v.y(), v.z());
}
```

However, if you must choose one: **Eigen** for serious vehicle simulation work. You can always write simple conversion functions for the rendering layer, and the physics quality will be worth it.

---

# ECS Libraries in Rust

Yes! Rust has several excellent ECS libraries, and the ecosystem is actually very mature. Here are the main options:

## Bevy ECS - Most Popular

**Bevy ECS** (part of the Bevy game engine, but usable standalone) is the most widely adopted:

```rust
use bevy_ecs::prelude::*;

// Components
#[derive(Component)]
struct Transform {
    position: Vec3,
    rotation: Quat,
}

#[derive(Component)]
struct Physics {
    velocity: Vec3,
    mass: f32,
}

#[derive(Component)]
struct Engine {
    rpm: f32,
    throttle: f32,
}

// System
fn update_physics(
    time: Res<Time>,
    mut query: Query<(&mut Transform, &mut Physics)>
) {
    for (mut transform, mut physics) in query.iter_mut() {
        transform.position += physics.velocity * time.delta_seconds();
        physics.velocity *= 0.99; // drag
    }
}

// Setup
fn main() {
    let mut world = World::new();
    
    // Spawn vehicles
    for i in 0..1000 {
        world.spawn((
            Transform { 
                position: Vec3::new(i as f32 * 5.0, 0.0, 0.0),
                rotation: Quat::IDENTITY 
            },
            Physics { 
                velocity: Vec3::ZERO,
                mass: 1500.0 
            },
            Engine { 
                rpm: 0.0,
                throttle: 0.0 
            },
        ));
    }
    
    // Run systems
    let mut schedule = Schedule::default();
    schedule.add_systems(update_physics);
}
```

**Pros**: 
- Excellent ergonomics with Rust's type system
- Great performance
- Active development and large community
- Comprehensive documentation
- Built-in scheduling and parallel execution

## Hecs - Lightweight Alternative

**Hecs** is a minimal, no-nonsense ECS similar to EnTT's philosophy:

```rust
use hecs::*;

fn main() {
    let mut world = World::new();
    
    // Spawn entities
    let entities: Vec<_> = (0..1000)
        .map(|i| {
            world.spawn((
                Transform::default(),
                Physics::default(),
            ))
        })
        .collect();
    
    // Query and update
    for (id, (transform, physics)) in world.query_mut::<(&mut Transform, &mut Physics)>() {
        transform.position += physics.velocity * 0.016;
    }
}
```

**Pros**:
- Very fast compile times
- Minimal API surface
- Excellent performance
- Low overhead

## Specs - The Veteran

**Specs** was the first major Rust ECS, inspired by C++ libraries:

```rust
use specs::prelude::*;

// Components need to implement Component
#[derive(Component)]
struct Transform { /* ... */ }

#[derive(Component)]
struct Physics { /* ... */ }

// Systems implement System trait
struct PhysicsSystem;

impl<'a> System<'a> for PhysicsSystem {
    type SystemData = (
        WriteStorage<'a, Transform>,
        WriteStorage<'a, Physics>,
    );
    
    fn run(&mut self, (mut transforms, mut physics): Self::SystemData) {
        for (transform, physics) in (&mut transforms, &mut physics).join() {
            transform.position += physics.velocity * 0.016;
        }
    }
}
```

**Pros**:
- Mature and battle-tested
- Parallel execution built-in
- Good performance

**Cons**:
- Somewhat more complex API
- Less active development than Bevy

## My Recommendation for Vehicle Simulation

**Use Bevy ECS** because:

1. **Modern Rust patterns**: Leverages Rust's type system beautifully with queries and system parameters

2. **Performance**: Archetype-based storage (like EnTT) with excellent cache locality

3. **Parallel by default**: Systems automatically run in parallel when possible

4. **Growing ecosystem**: Tons of plugins and community support

5. **Query flexibility**: Powerful query syntax for complex component combinations

```rust
// Complex queries are elegant in Bevy
fn update_ai_vehicles(
    mut query: Query
        (&mut Engine, &Transform, &Physics),
        (With<AIController>, Without<PlayerControlled>)
    >
) {
    for (mut engine, transform, physics) in query.iter_mut() {
        // Only AI-controlled, non-player vehicles
    }
}
```

## Comparison to EnTT

Feature-wise, **Bevy ECS** is most similar to EnTT but with better ergonomics thanks to Rust:

- ✅ SoA storage (archetypes)
- ✅ Type-safe queries
- ✅ Zero-cost abstractions
- ✅ Excellent cache performance
- ✅ Compile-time component validation
- ✅ No runtime overhead

Plus Rust-specific benefits:
- Memory safety guaranteed at compile time
- No data races in parallel systems
- Better error messages

If you want something even closer to EnTT's minimal philosophy, go with **Hecs**. But for a full vehicle simulation, Bevy ECS's extra features (scheduling, change detection, events) will save you time.

---

# Bevy ECS vs EnTT Performance Comparison

This is a great question, but the answer is nuanced. Let me break down what we know from benchmarks and architectural differences.

## What Benchmarks Show

**EnTT is generally faster** in raw microbenchmarks, but the margins vary significantly based on the workload:

**Simple iteration** (single component): EnTT is 5-20% faster
**Multi-component queries**: EnTT is 10-30% faster  
**Entity creation/destruction**: EnTT is significantly faster (2-3x)
**Random access**: EnTT is faster
**Complex queries with filters**: Margins narrow, sometimes even

## Why EnTT Tends to Be Faster

**1. Mature C++ optimizations**: EnTT has years of performance tuning and leverages C++ template metaprogramming aggressively for zero-cost abstractions.

**2. Storage model**: EnTT uses sparse sets with packed arrays, which is extremely efficient for iteration and random access.

**3. Groups**: EnTT's groups allow you to manually optimize hot paths by controlling memory layout precisely.

**4. No overhead**: Minimal abstraction layers - you're very close to raw memory manipulation.

```cpp
// EnTT: Direct memory access possible
auto view = registry.view<Transform, Physics>();
for (auto entity : view) {
    auto [transform, physics] = view.get<Transform, Physics>(entity);
    // Extremely tight loop, excellent codegen
}
```

## Why Bevy ECS Is Competitive

**1. Archetype storage**: Bevy groups entities by component combinations, which provides excellent cache locality when many entities share the same components (common in vehicle fleets).

**2. Modern design**: Built from the ground up with lessons learned from EnTT and other ECS libraries.

**3. Parallel execution**: Built-in parallelism that's harder to achieve safely in C++.

```rust
// Bevy: Automatic parallelization
fn update_vehicles(
    mut query: Query<(&mut Transform, &Physics)>
) {
    // This can automatically run in parallel across CPU cores
    query.par_iter_mut().for_each_mut(|(mut transform, physics)| {
        transform.position += physics.velocity * 0.016;
    });
}
```

## Vehicle Simulation Specific Considerations

For a **fleet of similar vehicles**, the comparison becomes interesting:

### Scenario 1: Homogeneous Fleet (All vehicles have same components)

**Bevy ECS would likely be comparable or even faster** because:

- All vehicles live in the same archetype = perfect cache locality
- Parallel iteration is trivial and safe
- No component checks needed during iteration

```rust
// All 1000 vehicles in one archetype = one contiguous memory block
world.spawn_batch((0..1000).map(|_| (
    Transform::default(),
    Physics::default(),
    Engine::default(),
    Wheel::default(),
)));
```

### Scenario 2: Heterogeneous Fleet (Different vehicle types)

**EnTT might have an edge** because:

- Sparse set iteration doesn't care about archetype fragmentation
- More efficient when entities have varying component sets
- Groups let you optimize specific combinations

```cpp
// EnTT handles this efficiently even with different component sets
auto basic_cars = registry.view<Transform, Physics>();
auto race_cars = registry.view<Transform, Physics, Aerodynamics>();
```

### Scenario 3: Dynamic Component Addition/Removal

**EnTT is significantly faster** because:

- Adding/removing components doesn't trigger archetype moves
- Bevy must move entities between archetypes when components change

```cpp
// EnTT: Fast component addition
registry.emplace<DamageComponent>(vehicle, 50.0f); // O(1)
```

```rust
// Bevy: Requires moving entity to new archetype
commands.entity(vehicle).insert(DamageComponent { value: 50.0 }); // More expensive
```

## Real-World Performance Factors

The practical difference in a vehicle simulation depends on:

**1. System complexity**: Complex physics calculations dwarf ECS overhead. If each vehicle update takes 100μs, 10μs ECS difference is negligible.

**2. Parallelization**: Bevy's safe parallelism can easily give you 4-8x speedup on multi-core systems, overwhelming any single-threaded advantage EnTT has.

**3. Development velocity**: Rust's safety guarantees prevent entire classes of bugs that can cause performance issues in C++ (data races, memory corruption).

## Practical Benchmark Example

Here's what you might see in a real vehicle simulation with 10,000 vehicles:

```
Single-threaded iteration (10k vehicles):
EnTT:     0.8ms per frame
Bevy ECS: 1.0ms per frame

Multi-threaded iteration (10k vehicles, 8 cores):
EnTT:     0.8ms per frame (requires manual parallelization)
Bevy ECS: 0.15ms per frame (automatic parallelization)

With complex physics (50μs per vehicle):
EnTT:     500ms per frame
Bevy ECS: 500ms per frame
(ECS overhead becomes irrelevant)
```

## My Verdict for Vehicle Simulation

**EnTT is faster** in raw ECS operations, but **Bevy ECS might be faster overall** because:

1. **Parallelism**: Vehicle simulations are embarrassingly parallel. Bevy's safe parallelism is a huge win.

2. **Fleet uniformity**: Vehicle fleets typically have homogeneous entities (all cars have similar components), which plays to Bevy's archetype strengths.

3. **Real bottlenecks**: Physics calculations, collision detection, and rendering will dominate performance - ECS overhead is typically <5% of frame time.

4. **Safety**: No risk of data races, iterator invalidation, or memory corruption that could tank performance in C++.

## When to Choose Each

**Choose EnTT if**:
- You need absolute maximum single-threaded performance
- You're adding/removing components frequently
- You have highly heterogeneous entities
- You're already in C++ and deeply optimized

**Choose Bevy ECS if**:
- You want safe automatic parallelism
- You have many similar entities (vehicle fleets!)
- Development velocity and safety matter
- You can leverage Rust's ecosystem

For a vehicle simulation specifically, I'd lean toward **Bevy ECS** because the parallelism benefits and safety guarantees outweigh the raw iteration speed advantage of EnTT, and the physics calculations will dominate performance anyway.

---

# Using C++ Libraries with Bevy ECS

Yes, you can use C++ libraries like OMPL and RVO2 with Bevy ECS, but there are important considerations about performance penalties and integration approaches.

## Integration Methods

### 1. FFI (Foreign Function Interface) - Direct Approach

You can call C++ libraries from Rust using FFI bindings:

```rust
// Using cxx crate for safe C++ interop
#[cxx::bridge]
mod ffi {
    unsafe extern "C++" {
        include!("rvo2/RVO.h");
        
        type RVOSimulator;
        
        fn createSimulator() -> UniquePtr<RVOSimulator>;
        fn addAgent(sim: Pin<&mut RVOSimulator>, 
                   position: &Vector2) -> i32;
        fn setAgentPrefVelocity(sim: Pin<&mut RVOSimulator>,
                               agent_id: i32,
                               velocity: &Vector2);
        fn doStep(sim: Pin<&mut RVOSimulator>, dt: f32);
    }
}

// In your Bevy system
fn rvo_collision_avoidance(
    mut query: Query<(&Transform, &mut Physics, &AgentId)>,
    mut rvo_sim: ResMut<RVOSimulatorResource>
) {
    // Update RVO with current positions
    for (transform, physics, agent_id) in query.iter() {
        let pos = Vector2 { 
            x: transform.position.x, 
            y: transform.position.z 
        };
        ffi::setAgentPrefVelocity(
            rvo_sim.0.pin_mut(), 
            agent_id.0, 
            &desired_velocity
        );
    }
    
    // Step RVO simulation
    ffi::doStep(rvo_sim.0.pin_mut(), 0.016);
    
    // Apply results back to entities
    for (transform, mut physics, agent_id) in query.iter_mut() {
        let new_velocity = ffi::getAgentVelocity(
            rvo_sim.0.pin_mut(), 
            agent_id.0
        );
        physics.velocity.x = new_velocity.x;
        physics.velocity.z = new_velocity.y;
    }
}
```

### 2. Existing Rust Bindings

Check if bindings already exist:
- **RVO2**: Look for `rvo2-rs` or similar crates (some exist but may be incomplete)
- **OMPL**: No mature Rust bindings, would need to create them

### 3. Pure Rust Alternatives

Consider native Rust implementations:
- **Pathfinding**: `pathfinding` crate (A*, Dijkstra, etc.)
- **Collision avoidance**: Implement RVO algorithm in Rust (it's not that complex)
- **Motion planning**: `ncollide` + custom planning algorithms

## Performance Penalty Analysis

### Where Penalties Come From

**1. FFI Boundary Crossing**
```rust
// Each call has overhead
for entity in entities.iter() {
    // This crosses FFI boundary - small cost per call
    let result = cpp_function(entity.data);
}
```

**Cost**: ~5-20 nanoseconds per call (negligible for planning, significant if called millions of times)

**2. Data Marshalling**
```rust
// Converting Rust types to C++ types
struct RustVehicle {
    position: Vec3,  // Rust layout
}

// Must convert to C++ layout
struct CppVehicle {
    float x, y, z;   // C++ layout
}
```

**Cost**: Depends on data complexity. Simple types (floats, ints) = zero cost. Complex types = copying overhead.

**3. Memory Layout Mismatch**

Bevy ECS stores components in Rust memory (SoA). C++ libraries expect their own data structures.

```rust
// Inefficient: Copy all data to C++ structures
fn inefficient_approach(query: Query<(&Transform, &Physics)>) {
    let mut cpp_vehicles = Vec::new();
    
    // Copy data to C++ format
    for (transform, physics) in query.iter() {
        cpp_vehicles.push(CppVehicle {
            x: transform.position.x,
            y: transform.position.y,
            z: transform.position.z,
            vx: physics.velocity.x,
            vy: physics.velocity.y,
            vz: physics.velocity.z,
        });
    }
    
    // Call C++ function with copied data
    unsafe { cpp_process_vehicles(cpp_vehicles.as_ptr(), cpp_vehicles.len()) };
    
    // Copy results back - another overhead!
}
```

## Realistic Performance Impact

### For Pathfinding (OMPL)

**Low penalty** because:
- Pathfinding is typically not called every frame
- It's computationally expensive anyway (milliseconds per query)
- FFI overhead is tiny compared to planning time

```rust
// Called once per second or on-demand
fn plan_path_system(
    mut query: Query<(&Transform, &Goal), With<NeedsNewPath>>,
    ompl_planner: Res<OMPLPlanner>
) {
    for (transform, goal) in query.iter() {
        // FFI call overhead: ~20ns
        // Path planning time: ~5-50ms
        // Overhead percentage: ~0.0004%
        let path = ompl_planner.plan(
            transform.position,
            goal.position
        );
    }
}
```

### For Collision Avoidance (RVO2)

**Medium penalty** because:
- Called every frame
- But operates on relatively small agent counts
- RVO2 is O(n²) in agents, FFI overhead is O(n)

```rust
// For 100 vehicles, 60 FPS
// FFI calls per second: 100 * 60 = 6,000
// FFI overhead: 6,000 * 20ns = 0.12ms per second
// RVO2 computation: ~1-5ms per frame
// Overhead percentage: ~2-10%
```

## Optimization Strategies

### 1. Batch FFI Calls

```rust
// Bad: Call FFI for each entity
for entity in entities.iter() {
    unsafe { cpp_update_single(entity.id, entity.position); }
}

// Good: Call FFI once with all data
let positions: Vec<_> = entities.iter()
    .map(|e| e.position)
    .collect();
unsafe { cpp_update_batch(positions.as_ptr(), positions.len()); }
```

### 2. Use Shared Memory

```rust
// Keep C++ simulator as persistent resource
#[derive(Resource)]
struct RVOSimulator {
    simulator: UniquePtr<CppRVOSimulator>,
}

// Update in-place, minimal copying
fn update_rvo(
    mut sim: ResMut<RVOSimulator>,
    query: Query<(&AgentId, &Transform, &DesiredVelocity)>
) {
    // Only update changed agents (use change detection)
    for (agent_id, transform, desired_vel) in query.iter() {
        sim.simulator.set_agent_position(agent_id.0, transform.position);
        sim.simulator.set_preferred_velocity(agent_id.0, desired_vel.0);
    }
    
    sim.simulator.do_step(0.016);
}
```

### 3. Run C++ in Separate Thread

```rust
use std::sync::mpsc;

// Offload heavy C++ computation to separate thread
fn async_path_planning(
    mut query: Query<(Entity, &Transform, &Goal), With<NeedsPath>>,
    planner_sender: Res<PlannerSender>,
) {
    for (entity, transform, goal) in query.iter() {
        // Send planning request to C++ thread
        planner_sender.send(PathRequest {
            entity,
            start: transform.position,
            goal: goal.position,
        });
    }
}

fn receive_paths(
    mut commands: Commands,
    planner_receiver: Res<PlannerReceiver>,
) {
    // Receive completed paths from C++ thread
    while let Ok(result) = planner_receiver.try_recv() {
        commands.entity(result.entity)
            .insert(Path { waypoints: result.path });
    }
}
```

## Real-World Example: Vehicle Simulation

Here's a realistic architecture:

```rust
// Resource wrapping C++ libraries
#[derive(Resource)]
struct MotionPlanningSystem {
    rvo_sim: UniquePtr<RVOSimulator>,
    ompl_planner: UniquePtr<OMPLPlanner>,
}

// Component to link Bevy entity to C++ agent
#[derive(Component)]
struct RVOAgentId(i32);

// System running at 60 FPS
fn collision_avoidance_system(
    mut motion_planning: ResMut<MotionPlanningSystem>,
    mut query: Query<(&Transform, &mut Physics, &RVOAgentId, &DesiredVelocity)>,
) {
    let start = std::time::Instant::now();
    
    // Update RVO simulator (FFI calls)
    for (transform, physics, agent_id, desired_vel) in query.iter() {
        motion_planning.rvo_sim.set_agent_position(
            agent_id.0,
            transform.position.xz(), // Convert to 2D
        );
        motion_planning.rvo_sim.set_preferred_velocity(
            agent_id.0,
            desired_vel.0.xz(),
        );
    }
    
    // Step simulation (expensive, but done in C++)
    motion_planning.rvo_sim.do_step(0.016);
    
    // Apply results back
    for (transform, mut physics, agent_id, desired_vel) in query.iter_mut() {
        let new_velocity = motion_planning.rvo_sim.get_agent_velocity(agent_id.0);
        physics.velocity.x = new_velocity.x;
        physics.velocity.z = new_velocity.y;
    }
    
    let elapsed = start.elapsed();
    // For 100 vehicles: typically 0.5-2ms including FFI overhead
}

// Path planning (called infrequently)
fn path_planning_system(
    motion_planning: Res<MotionPlanningSystem>,
    mut query: Query<(&Transform, &Goal, &mut Path), With<NeedsReplanning>>,
) {
    for (transform, goal, mut path) in query.iter_mut() {
        // This is expensive (5-50ms) but FFI overhead is <1%
        let new_path = motion_planning.ompl_planner.plan(
            transform.position,
            goal.position,
            5.0, // timeout
        );
        
        if let Some(waypoints) = new_path {
            path.waypoints = waypoints;
        }
    }
}
```

## Measured Performance

For a typical vehicle simulation with **100-1000 vehicles**:

| Operation | Time Without FFI | Time With FFI | Penalty |
|-----------|-----------------|---------------|---------|
| RVO2 collision avoidance (100 agents) | N/A (C++ only) | 0.5-2ms | ~0.1ms overhead |
| OMPL path planning (single query) | N/A (C++ only) | 5-50ms | ~0.001ms overhead |
| ECS iteration (1000 entities) | 0.05ms | 0.05ms | 0% (no FFI) |

**Bottom line**: FFI overhead is typically **1-5%** of total computation time for these use cases.

## My Recommendation

**Yes, use C++ libraries for OMPL and RVO2** because:

1. **Proven algorithms**: These libraries are battle-tested and correct
2. **Performance**: The algorithms themselves are expensive; FFI overhead is negligible
3. **Time-to-market**: Building equivalent Rust implementations would take months
4. **Integration is straightforward**: Use `cxx` crate for safe bindings

**Optimization tips**:
- Batch FFI calls where possible
- Keep C++ objects as Bevy resources (avoid recreation)
- Run expensive planning in separate threads
- Only update changed data (use Bevy's change detection)

The performance penalty will be **<5% in practice**, which is an excellent trade-off for using mature, correct implementations.

---

# SIMD in Bevy ECS

The answer is **nuanced** - Bevy uses SIMD selectively, but not everywhere you might expect.

## What Bevy Uses for Math

Bevy uses **glam** as its math library, which has sophisticated SIMD support:

```rust
// Bevy re-exports glam types
use bevy::prelude::*;

// These are glam types under the hood
let position = Vec3::new(1.0, 2.0, 3.0);
let velocity = Vec3::new(0.1, 0.0, 0.2);
let result = position + velocity; // May use SIMD
```

## SIMD Support in glam

**glam** provides multiple implementations depending on the target:

### Scalar (No SIMD)
```rust
// On platforms without SIMD or when disabled
pub struct Vec3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}
```

### SIMD (SSE2, AVX, NEON)
```rust
// On x86_64 with SSE2 (most common)
pub struct Vec3 {
    // Uses 128-bit SIMD register
    // Stores (x, y, z, w) where w is padding
    #[repr(C, align(16))]
    __m128: core::arch::x86_64::__m128,
}
```

## When SIMD Is Actually Used

### ✅ SIMD IS Used For:

**1. Individual vector/matrix operations**
```rust
// These operations use SIMD intrinsics
let a = Vec3::new(1.0, 2.0, 3.0);
let b = Vec3::new(4.0, 5.0, 6.0);
let c = a + b;           // SIMD addition
let d = a.dot(b);        // SIMD dot product
let e = a.cross(b);      // SIMD cross product
let f = a * 2.0;         // SIMD scalar multiplication

// Matrix operations
let mat = Mat4::from_rotation_translation(rotation, translation);
let transformed = mat * point; // SIMD matrix-vector multiply
```

**2. Transform hierarchies**
```rust
// Bevy's transform propagation uses SIMD
fn propagate_transforms(
    mut query: Query<(&mut GlobalTransform, &Transform)>
) {
    for (mut global, local) in query.iter_mut() {
        // These operations benefit from SIMD
        global.translation = local.translation;
        global.rotation = local.rotation;
        global.scale = local.scale;
    }
}
```

### ❌ SIMD IS NOT Used For:

**1. Iteration across multiple entities**
```rust
// This does NOT automatically vectorize across entities
fn update_positions(
    mut query: Query<(&mut Transform, &Velocity)>
) {
    for (mut transform, velocity) in query.iter_mut() {
        // Each entity processed individually
        // SIMD used within this operation, but not across entities
        transform.translation += velocity.0 * 0.016;
    }
}
```

This is **scalar processing across entities** - each entity is processed one at a time, even though each individual Vec3 operation might use SIMD internally.

**2. Batch operations across components**

Bevy doesn't automatically convert this:
```rust
// Processes entities one-by-one
for entity in entities {
    entity.position.x += entity.velocity.x * dt;
    entity.position.y += entity.velocity.y * dt;
    entity.position.z += entity.velocity.z * dt;
}
```

Into this SIMD-across-entities pattern:
```rust
// Process 4 entities at once (hypothetical)
for chunk in entities.chunks(4) {
    // Load 4 x-positions into one SIMD register
    let pos_x = load_simd(&chunk.positions.x);
    let vel_x = load_simd(&chunk.velocities.x);
    let result_x = pos_x + vel_x * dt;
    store_simd(&chunk.positions.x, result_x);
}
```

## Why Not SIMD Across Entities?

**1. Type system limitations**: Bevy's query system works with individual entities, not batches.

**2. Component layout**: While components are stored contiguously, the ECS abstraction doesn't expose them as raw arrays for SIMD batching.

**3. Complexity**: Auto-vectorization across entities would require:
   - Compile-time batch size determination
   - Handling remainder entities
   - Complex iterator implementations
   - Breaking Rust's type safety guarantees

## Performance Comparison

Here's what you actually get:

```rust
// Single entity update (with glam SIMD)
fn update_one_entity(transform: &mut Transform, velocity: &Vec3, dt: f32) {
    // Uses SIMD: processes x,y,z together in one instruction
    transform.translation += *velocity * dt;
    // ~1-2 CPU cycles for the vector operation
}

// 1000 entities
fn update_all_entities(query: Query<(&mut Transform, &Velocity)>) {
    for (mut transform, velocity) in query.iter() {
        transform.translation += velocity.0 * dt;
    }
    // ~1000-2000 CPU cycles total (1-2 per entity)
    // NOT ~250 cycles (if we did 4-wide SIMD across entities)
}
```

## Getting More SIMD Performance

If you need true SIMD-across-entities, you have options:

### 1. Manual Batching

```rust
use std::simd::*;

fn simd_update_positions(
    query: Query<(&mut Transform, &Velocity)>
) {
    let mut positions_x = Vec::new();
    let mut positions_y = Vec::new();
    let mut positions_z = Vec::new();
    let mut velocities_x = Vec::new();
    let mut velocities_y = Vec::new();
    let mut velocities_z = Vec::new();
    
    // Collect into SoA layout
    for (transform, velocity) in query.iter() {
        positions_x.push(transform.translation.x);
        positions_y.push(transform.translation.y);
        positions_z.push(transform.translation.z);
        velocities_x.push(velocity.0.x);
        velocities_y.push(velocity.0.y);
        velocities_z.push(velocity.0.z);
    }
    
    // Process in SIMD chunks
    for i in (0..positions_x.len()).step_by(8) {
        let pos_x = f32x8::from_slice(&positions_x[i..]);
        let vel_x = f32x8::from_slice(&velocities_x[i..]);
        let result = pos_x + vel_x * f32x8::splat(0.016);
        result.copy_to_slice(&mut positions_x[i..]);
    }
    
    // Write back (expensive!)
    // This is why it's often not worth it
}
```

**Problem**: The gather/scatter overhead often negates SIMD benefits!

### 2. Use Parallel Iteration Instead

```rust
use rayon::prelude::*;

fn parallel_update(
    query: Query<(&mut Transform, &Velocity)>
) {
    // This is usually faster than manual SIMD
    query.par_iter_mut().for_each(|(mut transform, velocity)| {
        transform.translation += velocity.0 * 0.016;
    });
}
```

Multi-core parallelism often gives better speedup than SIMD for this pattern (4-8x vs 2-4x).

### 3. Custom Storage for Hot Paths

```rust
// For critical inner loops, use custom SoA storage
#[derive(Resource)]
struct VehiclePhysicsData {
    positions_x: Vec<f32>,
    positions_y: Vec<f32>,
    positions_z: Vec<f32>,
    velocities_x: Vec<f32>,
    velocities_y: Vec<f32>,
    velocities_z: Vec<f32>,
    entity_ids: Vec<Entity>,
}

fn simd_optimized_physics(
    mut physics_data: ResMut<VehiclePhysicsData>,
) {
    use std::simd::*;
    
    let count = physics_data.positions_x.len();
    let chunks = count / 8;
    
    for i in 0..chunks {
        let idx = i * 8;
        let pos_x = f32x8::from_slice(&physics_data.positions_x[idx..]);
        let vel_x = f32x8::from_slice(&physics_data.velocities_x[idx..]);
        let result = pos_x + vel_x * f32x8::splat(0.016);
        result.copy_to_slice(&mut physics_data.positions_x[idx..]);
        
        // Repeat for y and z
    }
    
    // Sync back to ECS when needed
}
```

This gives true SIMD-across-entities but requires maintaining parallel data structures.

## Practical Recommendations for Vehicle Simulation

**1. Trust glam's SIMD** - It's already optimized for per-entity math:
```rust
// This is already well-optimized
fn update_vehicle(mut query: Query<(&mut Transform, &Physics)>) {
    for (mut transform, physics) in query.iter() {
        transform.translation += physics.velocity * 0.016; // SIMD within
    }
}
```

**2. Use parallelism for throughput**:
```rust
// Better than manual SIMD in most cases
query.par_iter_mut().for_each(|(mut transform, physics)| {
    transform.translation += physics.velocity * 0.016;
});
```

**3. Reserve manual SIMD for heavy math**:
```rust
// For complex per-vehicle calculations
fn compute_tire_forces(tire_state: &TireState) -> Vec3 {
    // Complex math here benefits from glam's SIMD
    let lateral_force = tire_state.slip_angle.sin() * tire_state.normal_force;
    let longitudinal_force = tire_state.slip_ratio.cos() * tire_state.normal_force;
    Vec3::new(longitudinal_force, 0.0, lateral_force)
}
```

**4. Profile before optimizing**:
Most vehicle simulations are bottlenecked by:
- Physics integration complexity (not throughput)
- Collision detection
- Rendering
- Not by the ECS iteration itself

## Summary

**Bevy DOES use SIMD**, but only within individual vector/matrix operations via glam, not across multiple entities. For vehicle simulation:

- ✅ Each vehicle's math operations use SIMD (glam)
- ❌ Processing multiple vehicles simultaneously with SIMD requires manual work
- ✅ Parallel iteration is usually more effective than manual SIMD
- ✅ glam's SIMD is sufficient for most use cases

The performance is still excellent - Bevy can easily handle thousands of vehicles at 60+ FPS. If you need more, multi-threading via `par_iter()` is your best bet before considering manual SIMD optimization.

---

# Using big_brain (formerly leafwing-ai) for Fleet Simulation

I'll assume you're referring to **big_brain** (the behavior tree/utility AI library for Bevy), as leafwing studios makes several Bevy crates. Let me evaluate whether it makes sense for fleet simulation.

## What is big_brain?

**big_brain** is a Utility AI library for Bevy that provides:
- **Scorers**: Evaluate the desirability of actions
- **Actions**: Behaviors that execute when chosen
- **Thinkers**: Decision-making entities that select actions based on scores

It's more of a **Utility AI** system than a traditional behavior tree, though it can be used similarly.

## When It Makes Sense

### ✅ Good Use Cases for Fleet Simulation

**1. Heterogeneous vehicle behaviors**
```rust
use big_brain::prelude::*;

// Different vehicle types with different decision-making
#[derive(Component, Debug, Clone, ActionBuilder)]
struct SeekDestination;

#[derive(Component, Debug, Clone, ActionBuilder)]
struct AvoidTraffic;

#[derive(Component, Debug, Clone, ActionBuilder)]
struct YieldToPedestrian;

// Scorer: How urgent is reaching destination?
#[derive(Component, Debug, Clone, ScorerBuilder)]
struct DestinationUrgency;

fn destination_urgency_scorer(
    mut query: Query<(&Transform, &Destination, &mut Score), With<DestinationUrgency>>
) {
    for (transform, destination, mut score) in query.iter_mut() {
        let distance = transform.translation.distance(destination.position);
        let urgency = 1.0 - (distance / 1000.0).min(1.0);
        score.set(urgency);
    }
}

// Set up AI for a vehicle
fn spawn_ai_vehicle(mut commands: Commands) {
    commands.spawn((
        Transform::default(),
        Physics::default(),
        Destination { position: Vec3::new(100.0, 0.0, 100.0) },
        Thinker::build()
            .when(DestinationUrgency, SeekDestination)
            .when(TrafficDensity, AvoidTraffic)
            .when(PedestrianProximity, YieldToPedestrian)
    ));
}
```

**2. Dynamic decision-making**
- Emergency vehicles changing behavior (sirens on/off)
- Delivery vehicles adapting routes based on traffic
- Autonomous vehicles making complex driving decisions

**3. Individual agent intelligence**
When each vehicle needs to make its own decisions based on:
- Local perception (nearby vehicles, obstacles)
- Internal state (fuel, damage, cargo)
- Goals (delivery, patrol, pursuit)

### ❌ When It Doesn't Make Sense

**1. Simple reactive behaviors**
```rust
// Don't need big_brain for this:
fn simple_follow_path(
    mut query: Query<(&Transform, &mut Physics, &Path)>
) {
    for (transform, mut physics, path) in query.iter_mut() {
        if let Some(next_waypoint) = path.current_waypoint() {
            let direction = (next_waypoint - transform.translation).normalize();
            physics.velocity = direction * 10.0;
        }
    }
}
```

**2. Centralized fleet control**
If you're doing traffic simulation where a central system controls all vehicles:
```rust
// Central traffic controller doesn't need per-vehicle AI
#[derive(Resource)]
struct TrafficController {
    assignments: HashMap<Entity, Route>,
}

fn centralized_control(
    controller: Res<TrafficController>,
    mut query: Query<(Entity, &mut Physics)>
) {
    for (entity, mut physics) in query.iter_mut() {
        if let Some(route) = controller.assignments.get(&entity) {
            // Central system decides everything
            physics.velocity = route.compute_velocity();
        }
    }
}
```

**3. Performance-critical swarms**
For 1000+ vehicles with simple behaviors, big_brain's overhead might be unnecessary:
```rust
// More efficient for large fleets with simple rules
fn swarm_behavior(
    spatial_index: Res<SpatialIndex>,
    mut query: Query<(&Transform, &mut Physics)>
) {
    for (transform, mut physics) in query.iter_mut() {
        // Simple flocking rules
        let neighbors = spatial_index.query_radius(transform.translation, 10.0);
        let separation = compute_separation(neighbors);
        let alignment = compute_alignment(neighbors);
        physics.velocity = separation + alignment;
    }
}
```

## Performance Considerations

### Overhead Analysis

**big_brain adds per-frame cost**:
```rust
// Each Thinker evaluates scorers every frame (by default)
Thinker::build()
    .when(Scorer1, Action1)  // Evaluates Scorer1
    .when(Scorer2, Action2)  // Evaluates Scorer2
    .when(Scorer3, Action3)  // Evaluates Scorer3
```

For **100 vehicles** with 5 scorers each:
- **Scorer evaluations**: 100 × 5 = 500 per frame
- **Cost**: ~0.1-0.5ms if scorers are simple
- **Cost**: ~1-5ms if scorers do complex queries

This is **acceptable** for most simulations, but can become a bottleneck with:
- 1000+ vehicles
- Complex scorer logic
- Every-frame evaluation

### Optimization Strategies

**1. Throttle thinking frequency**
```rust
#[derive(Component)]
struct ThinkTimer(Timer);

fn throttled_thinking(
    time: Res<Time>,
    mut query: Query<(&mut ThinkTimer, &mut Thinker)>
) {
    for (mut timer, mut thinker) in query.iter_mut() {
        timer.0.tick(time.delta());
        
        if timer.0.just_finished() {
            // Only think every 0.5 seconds instead of every frame
            thinker.force_update();
        }
    }
}

fn spawn_efficient_vehicle(mut commands: Commands) {
    commands.spawn((
        Transform::default(),
        // Think 2 times per second instead of 60
        ThinkTimer(Timer::from_seconds(0.5, TimerMode::Repeating)),
        Thinker::build()
            .when(DestinationUrgency, SeekDestination)
    ));
}
```

**2. Spatial partitioning for scorers**
```rust
// Expensive: Check all vehicles
#[derive(Component, Debug, Clone, ScorerBuilder)]
struct NearbyTrafficScorer;

fn bad_traffic_scorer(
    query: Query<(&Transform, &mut Score), With<NearbyTrafficScorer>>,
    all_vehicles: Query<&Transform, Without<NearbyTrafficScorer>>
) {
    for (transform, mut score) in query.iter_mut() {
        let mut nearby_count = 0;
        // O(n²) - very expensive!
        for other in all_vehicles.iter() {
            if transform.translation.distance(other.translation) < 20.0 {
                nearby_count += 1;
            }
        }
        score.set(nearby_count as f32 / 10.0);
    }
}

// Better: Use spatial index
fn good_traffic_scorer(
    spatial_index: Res<SpatialIndex>,
    query: Query<(&Transform, &mut Score), With<NearbyTrafficScorer>>
) {
    for (transform, mut score) in query.iter_mut() {
        // O(log n) - much faster!
        let nearby = spatial_index.query_radius(transform.translation, 20.0);
        score.set(nearby.len() as f32 / 10.0);
    }
}
```

**3. Hybrid approach**
```rust
// Use big_brain for complex vehicles, simple logic for background traffic
fn spawn_fleet(mut commands: Commands) {
    // Important vehicles get AI
    for i in 0..10 {
        commands.spawn((
            Transform::default(),
            AIPriority::High,
            Thinker::build()
                .when(ComplexScorer1, ComplexAction1)
                .when(ComplexScorer2, ComplexAction2)
        ));
    }
    
    // Background traffic uses simple rules
    for i in 0..1000 {
        commands.spawn((
            Transform::default(),
            SimpleTrafficBehavior,
        ));
    }
}
```

## Practical Example: Delivery Fleet

Here's where big_brain shines:

```rust
use big_brain::prelude::*;

// Actions
#[derive(Component, Debug, Clone, ActionBuilder)]
struct DriveToPickup;

#[derive(Component, Debug, Clone, ActionBuilder)]
struct DriveToDelivery;

#[derive(Component, Debug, Clone, ActionBuilder)]
struct RefuelAtStation;

#[derive(Component, Debug, Clone, ActionBuilder)]
struct WaitForTrafficClear;

// Scorers
#[derive(Component, Debug, Clone, ScorerBuilder)]
struct HasPickupTask {
    weight: f32,
}

#[derive(Component, Debug, Clone, ScorerBuilder)]
struct FuelLevel {
    weight: f32,
}

#[derive(Component, Debug, Clone, ScorerBuilder)]
struct TrafficCongestion {
    weight: f32,
}

// Scorer systems
fn has_pickup_scorer(
    mut query: Query<(&DeliveryVehicle, &mut Score), With<HasPickupTask>>
) {
    for (vehicle, mut score) in query.iter_mut() {
        score.set(if vehicle.has_pickup() { 1.0 } else { 0.0 });
    }
}

fn fuel_level_scorer(
    mut query: Query<(&DeliveryVehicle, &mut Score), With<FuelLevel>>
) {
    for (vehicle, mut score) in query.iter_mut() {
        // Critical when fuel < 20%
        let urgency = if vehicle.fuel < 0.2 { 1.0 } else { 0.0 };
        score.set(urgency);
    }
}

fn traffic_scorer(
    spatial_index: Res<SpatialIndex>,
    mut query: Query<(&Transform, &mut Score), With<TrafficCongestion>>
) {
    for (transform, mut score) in query.iter_mut() {
        let nearby = spatial_index.query_radius(transform.translation, 30.0);
        let congestion = (nearby.len() as f32 / 20.0).min(1.0);
        score.set(congestion);
    }
}

// Action systems
fn drive_to_pickup_action(
    mut query: Query<(&mut Transform, &mut ActionState, &DeliveryVehicle), With<DriveToPickup>>
) {
    for (mut transform, mut state, vehicle) in query.iter_mut() {
        match *state {
            ActionState::Requested => {
                *state = ActionState::Executing;
            }
            ActionState::Executing => {
                // Navigate to pickup location
                let direction = (vehicle.pickup_location - transform.translation).normalize();
                transform.translation += direction * 0.5;
                
                if transform.translation.distance(vehicle.pickup_location) < 1.0 {
                    *state = ActionState::Success;
                }
            }
            _ => {}
        }
    }
}

// Spawn delivery vehicle with AI
fn spawn_delivery_vehicle(mut commands: Commands) {
    commands.spawn((
        Transform::default(),
        DeliveryVehicle {
            fuel: 1.0,
            pickup_location: Vec3::new(100.0, 0.0, 50.0),
            delivery_location: Vec3::new(-50.0, 0.0, -80.0),
        },
        // Think every 0.5 seconds
        ThinkTimer(Timer::from_seconds(0.5, TimerMode::Repeating)),
        Thinker::build()
            .when(
                HasPickupTask { weight: 1.0 },
                DriveToPickup
            )
            .when(
                FuelLevel { weight: 2.0 },  // Higher priority
                RefuelAtStation
            )
            .when(
                TrafficCongestion { weight: 0.5 },
                WaitForTrafficClear
            )
            .otherwise(DriveToDelivery)
    ));
}
```

## Alternative: Custom Decision System

For simpler cases, you might not need big_brain:

```rust
#[derive(Component)]
enum VehicleState {
    FollowingPath,
    AvoidingObstacle,
    Stopped,
}

fn simple_decision_system(
    mut query: Query<(&Transform, &mut VehicleState, &mut Physics, &Path)>,
    obstacles: Query<&Transform, With<Obstacle>>
) {
    for (transform, mut state, mut physics, path) in query.iter_mut() {
        // Check for obstacles
        let obstacle_nearby = obstacles.iter().any(|obs| {
            transform.translation.distance(obs.translation) < 5.0
        });
        
        // Simple state machine
        *state = if obstacle_nearby {
            VehicleState::AvoidingObstacle
        } else if path.is_complete() {
            VehicleState::Stopped
        } else {
            VehicleState::FollowingPath
        };
        
        // Execute behavior
        match *state {
            VehicleState::FollowingPath => {
                let target = path.current_waypoint();
                physics.velocity = (target - transform.translation).normalize() * 10.0;
            }
            VehicleState::AvoidingObstacle => {
                physics.velocity *= 0.5; // Slow down
            }
            VehicleState::Stopped => {
                physics.velocity = Vec3::ZERO;
            }
        }
    }
}
```

## My Recommendation

**Use big_brain if**:
- ✅ You have <500 vehicles with AI (or throttle thinking for more)
- ✅ Vehicles need complex, multi-factor decision making
- ✅ Behaviors change dynamically based on multiple conditions
- ✅ You want emergent, intelligent behavior
- ✅ Development time matters (big_brain is well-documented)

**Skip big_brain if**:
- ❌ You have 1000+ simple vehicles
- ❌ Behaviors are mostly reactive/rule-based
- ❌ You need absolute maximum performance
- ❌ Central coordination is preferred over individual intelligence

**Hybrid approach** (best for most cases):
- Use big_brain for hero vehicles, player-visible NPCs, or complex agents
- Use simple systems for background traffic/swarm behaviors
- Profile to ensure thinking costs don't dominate frame time

For a typical **delivery fleet simulation** with 50-200 vehicles making routing decisions, **big_brain is an excellent choice** - the code clarity and behavior flexibility are worth the modest performance cost.

---

# big_brain Integration with Bevy ECS

**Yes, big_brain plays extremely nicely with Bevy ECS** - in fact, it's designed specifically for Bevy and leverages ECS principles throughout. It's one of the best examples of a third-party Bevy plugin that feels native to the ecosystem.

## Why big_brain Fits Naturally

### 1. **Pure ECS Design**

big_brain doesn't fight against ECS - it embraces it completely:

```rust
use big_brain::prelude::*;
use bevy::prelude::*;

// Scorers are just components
#[derive(Component, Debug, Clone, ScorerBuilder)]
struct Thirst {
    thirst_per_second: f32,
}

// Actions are just components
#[derive(Component, Debug, Clone, ActionBuilder)]
struct Drink;

// Thinker is just a component that ties them together
fn spawn_vehicle(mut commands: Commands) {
    commands.spawn((
        // Your existing components
        Transform::default(),
        Physics::default(),
        // Add AI components alongside them
        Thinker::build()
            .when(Thirst { thirst_per_second: 0.1 }, Drink),
    ));
}
```

Everything is a component - no special "AI entity" types, no parallel data structures.

### 2. **Works with Bevy's Query System**

Scorers and actions are just regular Bevy systems:

```rust
// This is a normal Bevy system that works with any Query
fn thirst_scorer_system(
    time: Res<Time>,
    mut query: Query<(&Thirst, &mut Score)>
) {
    for (thirst, mut score) in query.iter_mut() {
        // Access any component on the entity
        score.set(thirst.thirst_per_second * time.elapsed_seconds());
    }
}

// Action systems have full access to the ECS world
fn drink_action_system(
    mut query: Query<(
        &Transform,           // Your components
        &mut Physics,         // Your components
        &mut ActionState,     // big_brain component
        &mut Thirst,         // Scorer component
    ), With<Drink>>
) {
    for (transform, mut physics, mut action_state, mut thirst) in query.iter_mut() {
        match *action_state {
            ActionState::Requested => {
                println!("Vehicle starting to refuel");
                *action_state = ActionState::Executing;
            }
            ActionState::Executing => {
                // Interact with your physics, transform, etc.
                physics.velocity *= 0.5; // Slow down while refueling
                thirst.thirst_per_second = 0.0;
                
                // Complete when done
                *action_state = ActionState::Success;
            }
            _ => {}
        }
    }
}
```

### 3. **Composes with Other Systems**

big_brain systems run alongside your existing systems without conflicts:

```rust
fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(BigBrainPlugin::new(PreUpdate)) // Runs in PreUpdate
        .add_systems(Update, (
            // Your existing systems
            physics_system,
            collision_system,
            rendering_system,
            // big_brain systems integrate seamlessly
            thirst_scorer_system,
            drink_action_system,
            // Can even query AI state in your systems
            debug_ai_decisions,
        ))
        .run();
}

// Your systems can read AI state
fn debug_ai_decisions(
    query: Query<(Entity, &ActionState, &Thinker)>
) {
    for (entity, action_state, thinker) in query.iter() {
        println!("Entity {:?} is executing: {:?}", entity, action_state);
    }
}
```

### 4. **Uses Change Detection**

big_brain leverages Bevy's change detection for efficiency:

```rust
// Only re-evaluate when components change
fn smart_scorer(
    // Only runs when Transform or TargetLocation changed
    query: Query
        (&Transform, &TargetLocation, &mut Score),
        Or<(Changed<Transform>, Changed<TargetLocation>)>
    >
) {
    for (transform, target, mut score) in query.iter_mut() {
        let distance = transform.translation.distance(target.position);
        score.set(1.0 / distance);
    }
}
```

### 5. **Hierarchical Entity Support**

Works naturally with Bevy's entity hierarchies:

```rust
// Parent vehicle with AI
commands.spawn((
    Transform::default(),
    Thinker::build().when(SeekTarget, DriveToTarget),
))
.with_children(|parent| {
    // Child entities (wheels, sensors, etc.)
    parent.spawn((
        Transform::default(),
        WheelComponent,
    ));
});

// AI can query children/parents
fn action_with_hierarchy(
    mut query: Query<(&Transform, &mut ActionState, &Children), With<DriveToTarget>>,
    wheels: Query<&WheelComponent>,
) {
    for (transform, mut state, children) in query.iter_mut() {
        // Access child components during action
        for &child in children.iter() {
            if let Ok(wheel) = wheels.get(child) {
                // Use wheel data in action logic
            }
        }
    }
}
```

## Practical Integration Example

Here's a complete vehicle fleet with big_brain and standard Bevy patterns:

```rust
use bevy::prelude::*;
use big_brain::prelude::*;

// Your existing components
#[derive(Component)]
struct Physics {
    velocity: Vec3,
    acceleration: Vec3,
}

#[derive(Component)]
struct Destination {
    position: Vec3,
}

#[derive(Component)]
struct FuelTank {
    current: f32,
    capacity: f32,
}

// big_brain Scorers
#[derive(Component, Debug, Clone, ScorerBuilder)]
struct DistanceToDestination;

#[derive(Component, Debug, Clone, ScorerBuilder)]
struct FuelUrgency;

// big_brain Actions
#[derive(Component, Debug, Clone, ActionBuilder)]
struct DriveToDestination;

#[derive(Component, Debug, Clone, ActionBuilder)]
struct RefuelAtStation;

// Scorer system - regular Bevy system
fn distance_scorer(
    mut query: Query<(&Transform, &Destination, &mut Score), With<DistanceToDestination>>
) {
    for (transform, destination, mut score) in query.iter_mut() {
        let distance = transform.translation.distance(destination.position);
        // Closer = higher score
        score.set(1.0 / (distance + 1.0));
    }
}

fn fuel_scorer(
    mut query: Query<(&FuelTank, &mut Score), With<FuelUrgency>>
) {
    for (fuel, mut score) in query.iter_mut() {
        let ratio = fuel.current / fuel.capacity;
        // Low fuel = high urgency
        score.set(if ratio < 0.2 { 1.0 } else { 0.0 });
    }
}

// Action system - regular Bevy system
fn drive_action(
    mut query: Query<(
        &Transform,
        &Destination,
        &mut Physics,
        &mut ActionState,
    ), With<DriveToDestination>>
) {
    for (transform, destination, mut physics, mut state) in query.iter_mut() {
        match *state {
            ActionState::Requested => {
                println!("Starting to drive");
                *state = ActionState::Executing;
            }
            ActionState::Executing => {
                // Use your existing physics system
                let direction = (destination.position - transform.translation).normalize();
                physics.acceleration = direction * 5.0;
                
                if transform.translation.distance(destination.position) < 1.0 {
                    *state = ActionState::Success;
                }
            }
            ActionState::Cancelled => {
                physics.acceleration = Vec3::ZERO;
                *state = ActionState::Failure;
            }
            _ => {}
        }
    }
}

fn refuel_action(
    mut query: Query<(
        &mut FuelTank,
        &mut Physics,
        &mut ActionState,
    ), With<RefuelAtStation>>,
    time: Res<Time>,
) {
    for (mut fuel, mut physics, mut state) in query.iter_mut() {
        match *state {
            ActionState::Requested => {
                println!("Refueling...");
                physics.velocity = Vec3::ZERO; // Stop while refueling
                *state = ActionState::Executing;
            }
            ActionState::Executing => {
                // Refuel over time
                fuel.current += 10.0 * time.delta_seconds();
                
                if fuel.current >= fuel.capacity {
                    fuel.current = fuel.capacity;
                    *state = ActionState::Success;
                    println!("Refueling complete");
                }
            }
            _ => {}
        }
    }
}

// Your existing physics system works unchanged
fn physics_system(
    time: Res<Time>,
    mut query: Query<(&mut Transform, &mut Physics)>
) {
    for (mut transform, mut physics) in query.iter_mut() {
        physics.velocity += physics.acceleration * time.delta_seconds();
        transform.translation += physics.velocity * time.delta_seconds();
        physics.velocity *= 0.98; // Drag
    }
}

// Spawn vehicles with AI
fn setup(mut commands: Commands) {
    for i in 0..10 {
        commands.spawn((
            // Your components
            Transform::from_xyz(i as f32 * 10.0, 0.0, 0.0),
            Physics {
                velocity: Vec3::ZERO,
                acceleration: Vec3::ZERO,
            },
            Destination {
                position: Vec3::new(100.0, 0.0, 100.0),
            },
            FuelTank {
                current: 50.0,
                capacity: 100.0,
            },
            // Add AI
            Thinker::build()
                .when(
                    FuelUrgency,
                    RefuelAtStation
                )
                .when(
                    DistanceToDestination,
                    DriveToDestination
                ),
        ));
    }
}

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(BigBrainPlugin::new(PreUpdate))
        .add_systems(Startup, setup)
        .add_systems(Update, (
            // Your existing systems
            physics_system,
            // big_brain scorer systems
            distance_scorer,
            fuel_scorer,
            // big_brain action systems
            drive_action,
            refuel_action,
        ))
        .run();
}
```

## Things That Work Great

### ✅ **Entity Queries in Actions**

Actions can query other entities freely:

```rust
fn avoid_collision_action(
    mut query: Query<(&Transform, &mut Physics, &mut ActionState), With<AvoidCollision>>,
    obstacles: Query<&Transform, (With<Obstacle>, Without<AvoidCollision>)>,
) {
    for (transform, mut physics, mut state) in query.iter_mut() {
        // Find nearest obstacle
        let nearest = obstacles.iter()
            .min_by_key(|obs| {
                transform.translation.distance(obs.translation) as i32
            });
            
        if let Some(obstacle) = nearest {
            let avoid_direction = (transform.translation - obstacle.translation).normalize();
            physics.acceleration = avoid_direction * 10.0;
        }
    }
}
```

### ✅ **Events Integration**

big_brain works with Bevy's event system:

```rust
#[derive(Event)]
struct CollisionEvent {
    entity: Entity,
}

fn collision_aware_action(
    mut events: EventReader<CollisionEvent>,
    mut query: Query<&mut ActionState>,
) {
    for event in events.read() {
        if let Ok(mut state) = query.get_mut(event.entity) {
            // Cancel action on collision
            *state = ActionState::Cancelled;
        }
    }
}
```

### ✅ **Resources Access**

Scorers and actions can use resources:

```rust
#[derive(Resource)]
struct TrafficDensity {
    grid: HashMap<IVec2, f32>,
}

fn traffic_aware_scorer(
    traffic: Res<TrafficDensity>,
    mut query: Query<(&Transform, &mut Score), With<TrafficAvoidance>>
) {
    for (transform, mut score) in query.iter_mut() {
        let grid_pos = IVec2::new(
            (transform.translation.x / 10.0) as i32,
            (transform.translation.z / 10.0) as i32,
        );
        
        let density = traffic.grid.get(&grid_pos).unwrap_or(&0.0);
        score.set(*density);
    }
}
```

### ✅ **Parallel System Support**

big_brain systems can run in parallel with proper queries:

```rust
fn parallel_scorers(
    mut scorer1: Query<(&Transform, &mut Score), (With<Scorer1>, Without<Scorer2>)>,
    mut scorer2: Query<(&Transform, &mut Score), (With<Scorer2>, Without<Scorer1>)>,
) {
    // These run in parallel automatically
    scorer1.par_iter_mut().for_each(|(transform, mut score)| {
        score.set(calculate_score1(transform));
    });
    
    scorer2.par_iter_mut().for_each(|(transform, mut score)| {
        score.set(calculate_score2(transform));
    });
}
```

## Potential Gotchas (Minor)

### ⚠️ **System Ordering**

You need to ensure big_brain runs at the right time:

```rust
// big_brain needs to run in PreUpdate to evaluate before your Update systems
App::new()
    .add_plugins(BigBrainPlugin::new(PreUpdate)) // Important!
    .add_systems(Update, your_systems)
```

### ⚠️ **ActionState Management**

Actions must properly manage their state:

```rust
// BAD: Forgot to set state
fn bad_action(mut query: Query<&mut ActionState, With<MyAction>>) {
    for mut state in query.iter_mut() {
        // Forgot to set state - action never completes!
        do_something();
    }
}

// GOOD: Always transition states
fn good_action(mut query: Query<&mut ActionState, With<MyAction>>) {
    for mut state in query.iter_mut() {
        match *state {
            ActionState::Requested => *state = ActionState::Executing,
            ActionState::Executing => {
                do_something();
                *state = ActionState::Success;
            }
            _ => {}
        }
    }
}
```

### ⚠️ **Score Normalization**

Keep scores in 0.0-1.0 range for consistent behavior:

```rust
// BAD: Unbounded scores
score.set(distance); // Could be 1000.0

// GOOD: Normalized
score.set((1.0 / (distance + 1.0)).min(1.0));
```

## Performance with Bevy ECS

big_brain leverages Bevy's performance characteristics:

```rust
// Efficient: Uses Bevy's archetype storage
fn efficient_scoring(
    // All entities with these exact components live in same archetype
    mut query: Query<(&Transform, &Velocity, &mut Score), With<MyScorer>>
) {
    // Cache-friendly iteration
    for (transform, velocity, mut score) in query.iter_mut() {
        score.set(calculate(transform, velocity));
    }
}
```

## Summary

**big_brain is exceptionally well-integrated with Bevy ECS**:

✅ Pure component-based design  
✅ Uses standard Bevy queries and systems  
✅ Works with change detection  
✅ Supports entity hierarchies  
✅ Integrates with events and resources  
✅ Can run systems in parallel  
✅ Respects Bevy's system ordering  
✅ No special entity types or data structures  
✅ Feels native to Bevy's API

**It's not a separate "AI system" bolted on** - it's a set of components and systems that play by all of Bevy's rules. If you understand Bevy ECS, you already understand 90% of big_brain.

For your vehicle fleet simulation, you can freely mix big_brain AI with your existing physics, rendering, and gameplay systems without any architectural friction.

---

# Integrating OMPL and RVO2 into Rust/Bevy Fleet App

This is a comprehensive integration challenge. Let me walk you through the complete setup for both libraries and how they fit into the decision process with big_brain.

## Part 1: Creating FFI Bindings

### Option A: Using `cxx` Crate (Recommended)

The `cxx` crate provides safe C++/Rust interop with excellent ergonomics.

**Project Structure:**
```
fleet_simulation/
├── Cargo.toml
├── build.rs
├── src/
│   ├── main.rs
│   ├── ffi/
│   │   ├── mod.rs
│   │   ├── ompl_bridge.rs
│   │   └── rvo2_bridge.rs
├── cpp/
│   ├── ompl_wrapper.h
│   ├── ompl_wrapper.cpp
│   ├── rvo2_wrapper.h
│   └── rvo2_wrapper.cpp
└── include/
    └── (OMPL and RVO2 headers)
```

### Step 1: Cargo.toml

```toml
[package]
name = "fleet_simulation"
version = "0.1.0"
edition = "2021"

[dependencies]
bevy = "0.14"
big_brain = "0.20"
cxx = "1.0"

[build-dependencies]
cxx-build = "1.0"

[lib]
crate-type = ["cdylib", "rlib"]
```

### Step 2: C++ Wrappers

**cpp/ompl_wrapper.h:**
```cpp
#pragma once
#include <memory>
#include <vector>
#include "rust/cxx.h"

// Forward declarations
namespace ompl {
    namespace base {
        class StateSpace;
        class SpaceInformation;
    }
    namespace geometric {
        class SimpleSetup;
    }
}

struct Vec3 {
    float x;
    float y;
    float z;
};

struct PathResult {
    std::vector<Vec3> waypoints;
    bool success;
};

class OMPLPlanner {
public:
    OMPLPlanner(Vec3 bounds_min, Vec3 bounds_max);
    ~OMPLPlanner();
    
    PathResult plan_path(Vec3 start, Vec3 goal, float timeout_seconds);
    void add_obstacle(Vec3 center, float radius);
    void clear_obstacles();

private:
    std::shared_ptr<ompl::base::StateSpace> space_;
    std::shared_ptr<ompl::base::SpaceInformation> si_;
    std::shared_ptr<ompl::geometric::SimpleSetup> ss_;
    
    struct Obstacle {
        Vec3 center;
        float radius;
    };
    std::vector<Obstacle> obstacles_;
};

std::unique_ptr<OMPLPlanner> create_ompl_planner(Vec3 bounds_min, Vec3 bounds_max);
```

**cpp/ompl_wrapper.cpp:**
```cpp
#include "ompl_wrapper.h"
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

OMPLPlanner::OMPLPlanner(Vec3 bounds_min, Vec3 bounds_max) {
    // Create 3D state space
    auto space = std::make_shared<ob::RealVectorStateSpace>(3);
    
    // Set bounds
    ob::RealVectorBounds bounds(3);
    bounds.setLow(0, bounds_min.x);
    bounds.setLow(1, bounds_min.y);
    bounds.setLow(2, bounds_min.z);
    bounds.setHigh(0, bounds_max.x);
    bounds.setHigh(1, bounds_max.y);
    bounds.setHigh(2, bounds_max.z);
    space->setBounds(bounds);
    
    space_ = space;
    si_ = std::make_shared<ob::SpaceInformation>(space_);
    
    // Set state validity checker (collision detection)
    si_->setStateValidityChecker([this](const ob::State *state) {
        auto *pos = state->as<ob::RealVectorStateSpace::StateType>();
        Vec3 point{
            static_cast<float>(pos->values[0]),
            static_cast<float>(pos->values[1]),
            static_cast<float>(pos->values[2])
        };
        
        // Check against obstacles
        for (const auto& obs : obstacles_) {
            float dx = point.x - obs.center.x;
            float dy = point.y - obs.center.y;
            float dz = point.z - obs.center.z;
            float dist_sq = dx*dx + dy*dy + dz*dz;
            if (dist_sq < obs.radius * obs.radius) {
                return false; // Collision
            }
        }
        return true; // Valid
    });
    
    si_->setup();
    
    ss_ = std::make_shared<og::SimpleSetup>(si_);
    ss_->setPlanner(std::make_shared<og::RRTstar>(si_));
    
    // Set optimization objective
    ss_->setOptimizationObjective(
        std::make_shared<ob::PathLengthOptimizationObjective>(si_)
    );
}

OMPLPlanner::~OMPLPlanner() = default;

PathResult OMPLPlanner::plan_path(Vec3 start, Vec3 goal, float timeout_seconds) {
    PathResult result;
    result.success = false;
    
    // Set start state
    ob::ScopedState<> start_state(space_);
    start_state[0] = start.x;
    start_state[1] = start.y;
    start_state[2] = start.z;
    
    // Set goal state
    ob::ScopedState<> goal_state(space_);
    goal_state[0] = goal.x;
    goal_state[1] = goal.y;
    goal_state[2] = goal.z;
    
    ss_->setStartAndGoalStates(start_state, goal_state);
    
    // Solve
    ob::PlannerStatus solved = ss_->solve(timeout_seconds);
    
    if (solved) {
        // Simplify and get path
        ss_->simplifySolution();
        auto path = ss_->getSolutionPath();
        
        result.success = true;
        result.waypoints.reserve(path.getStateCount());
        
        for (size_t i = 0; i < path.getStateCount(); ++i) {
            auto *state = path.getState(i)->as<ob::RealVectorStateSpace::StateType>();
            result.waypoints.push_back(Vec3{
                static_cast<float>(state->values[0]),
                static_cast<float>(state->values[1]),
                static_cast<float>(state->values[2])
            });
        }
    }
    
    ss_->clear();
    return result;
}

void OMPLPlanner::add_obstacle(Vec3 center, float radius) {
    obstacles_.push_back({center, radius});
}

void OMPLPlanner::clear_obstacles() {
    obstacles_.clear();
}

std::unique_ptr<OMPLPlanner> create_ompl_planner(Vec3 bounds_min, Vec3 bounds_max) {
    return std::make_unique<OMPLPlanner>(bounds_min, bounds_max);
}
```

**cpp/rvo2_wrapper.h:**
```cpp
#pragma once
#include <memory>
#include <vector>
#include "rust/cxx.h"

// Forward declaration
namespace RVO {
    class RVOSimulator;
}

struct Vec2 {
    float x;
    float y;
};

struct AgentParams {
    float neighbor_dist;
    size_t max_neighbors;
    float time_horizon;
    float time_horizon_obst;
    float radius;
    float max_speed;
    Vec2 velocity;
};

class RVO2Simulator {
public:
    RVO2Simulator();
    ~RVO2Simulator();
    
    size_t add_agent(Vec2 position, AgentParams params);
    void set_agent_pref_velocity(size_t agent_id, Vec2 velocity);
    void set_agent_position(size_t agent_id, Vec2 position);
    Vec2 get_agent_velocity(size_t agent_id) const;
    Vec2 get_agent_position(size_t agent_id) const;
    void do_step(float time_step);
    void remove_agent(size_t agent_id);
    size_t get_num_agents() const;

private:
    std::unique_ptr<RVO::RVOSimulator> sim_;
};

std::unique_ptr<RVO2Simulator> create_rvo2_simulator();
```

**cpp/rvo2_wrapper.cpp:**
```cpp
#include "rvo2_wrapper.h"
#include <RVO.h>

RVO2Simulator::RVO2Simulator() {
    sim_ = std::make_unique<RVO::RVOSimulator>();
}

RVO2Simulator::~RVO2Simulator() = default;

size_t RVO2Simulator::add_agent(Vec2 position, AgentParams params) {
    return sim_->addAgent(
        RVO::Vector2(position.x, position.y),
        params.neighbor_dist,
        params.max_neighbors,
        params.time_horizon,
        params.time_horizon_obst,
        params.radius,
        params.max_speed,
        RVO::Vector2(params.velocity.x, params.velocity.y)
    );
}

void RVO2Simulator::set_agent_pref_velocity(size_t agent_id, Vec2 velocity) {
    sim_->setAgentPrefVelocity(agent_id, RVO::Vector2(velocity.x, velocity.y));
}

void RVO2Simulator::set_agent_position(size_t agent_id, Vec2 position) {
    sim_->setAgentPosition(agent_id, RVO::Vector2(position.x, position.y));
}

Vec2 RVO2Simulator::get_agent_velocity(size_t agent_id) const {
    auto vel = sim_->getAgentVelocity(agent_id);
    return Vec2{static_cast<float>(vel.x()), static_cast<float>(vel.y())};
}

Vec2 RVO2Simulator::get_agent_position(size_t agent_id) const {
    auto pos = sim_->getAgentPosition(agent_id);
    return Vec2{static_cast<float>(pos.x()), static_cast<float>(pos.y())};
}

void RVO2Simulator::do_step(float time_step) {
    sim_->doStep();
}

void RVO2Simulator::remove_agent(size_t agent_id) {
    // RVO2 doesn't have direct remove, but you can track and skip
    // For simplicity, we'll just set velocity to zero
    sim_->setAgentPrefVelocity(agent_id, RVO::Vector2(0.0, 0.0));
}

size_t RVO2Simulator::get_num_agents() const {
    return sim_->getNumAgents();
}

std::unique_ptr<RVO2Simulator> create_rvo2_simulator() {
    return std::make_unique<RVO2Simulator>();
}
```

### Step 3: Rust Bridge Definitions

**src/ffi/ompl_bridge.rs:**
```rust
#[cxx::bridge]
pub mod ompl_ffi {
    #[derive(Debug, Clone)]
    struct Vec3 {
        x: f32,
        y: f32,
        z: f32,
    }

    #[derive(Debug)]
    struct PathResult {
        waypoints: Vec<Vec3>,
        success: bool,
    }

    unsafe extern "C++" {
        include!("cpp/ompl_wrapper.h");

        type OMPLPlanner;

        fn create_ompl_planner(bounds_min: Vec3, bounds_max: Vec3) -> UniquePtr<OMPLPlanner>;
        fn plan_path(self: Pin<&mut OMPLPlanner>, start: Vec3, goal: Vec3, timeout_seconds: f32) -> PathResult;
        fn add_obstacle(self: Pin<&mut OMPLPlanner>, center: Vec3, radius: f32);
        fn clear_obstacles(self: Pin<&mut OMPLPlanner>);
    }
}

impl From<bevy::prelude::Vec3> for ompl_ffi::Vec3 {
    fn from(v: bevy::prelude::Vec3) -> Self {
        Self { x: v.x, y: v.y, z: v.z }
    }
}

impl From<ompl_ffi::Vec3> for bevy::prelude::Vec3 {
    fn from(v: ompl_ffi::Vec3) -> Self {
        Self::new(v.x, v.y, v.z)
    }
}
```

**src/ffi/rvo2_bridge.rs:**
```rust
#[cxx::bridge]
pub mod rvo2_ffi {
    #[derive(Debug, Clone)]
    struct Vec2 {
        x: f32,
        y: f32,
    }

    #[derive(Debug, Clone)]
    struct AgentParams {
        neighbor_dist: f32,
        max_neighbors: usize,
        time_horizon: f32,
        time_horizon_obst: f32,
        radius: f32,
        max_speed: f32,
        velocity: Vec2,
    }

    unsafe extern "C++" {
        include!("cpp/rvo2_wrapper.h");

        type RVO2Simulator;

        fn create_rvo2_simulator() -> UniquePtr<RVO2Simulator>;
        fn add_agent(self: Pin<&mut RVO2Simulator>, position: Vec2, params: AgentParams) -> usize;
        fn set_agent_pref_velocity(self: Pin<&mut RVO2Simulator>, agent_id: usize, velocity: Vec2);
        fn set_agent_position(self: Pin<&mut RVO2Simulator>, agent_id: usize, position: Vec2);
        fn get_agent_velocity(self: &RVO2Simulator, agent_id: usize) -> Vec2;
        fn get_agent_position(self: &RVO2Simulator, agent_id: usize) -> Vec2;
        fn do_step(self: Pin<&mut RVO2Simulator>, time_step: f32);
        fn remove_agent(self: Pin<&mut RVO2Simulator>, agent_id: usize);
        fn get_num_agents(self: &RVO2Simulator) -> usize;
    }
}

impl Default for rvo2_ffi::AgentParams {
    fn default() -> Self {
        Self {
            neighbor_dist: 15.0,
            max_neighbors: 10,
            time_horizon: 2.0,
            time_horizon_obst: 2.0,
            radius: 2.0,
            max_speed: 10.0,
            velocity: rvo2_ffi::Vec2 { x: 0.0, y: 0.0 },
        }
    }
}
```

**src/ffi/mod.rs:**
```rust
pub mod ompl_bridge;
pub mod rvo2_bridge;

pub use ompl_bridge::ompl_ffi;
pub use rvo2_bridge::rvo2_ffi;
```

### Step 4: Build Script

**build.rs:**
```rust
fn main() {
    // Build OMPL wrapper
    cxx_build::bridge("src/ffi/ompl_bridge.rs")
        .file("cpp/ompl_wrapper.cpp")
        .include("include") // OMPL headers
        .flag_if_supported("-std=c++17")
        .compile("ompl_wrapper");

    // Build RVO2 wrapper
    cxx_build::bridge("src/ffi/rvo2_bridge.rs")
        .file("cpp/rvo2_wrapper.cpp")
        .include("include") // RVO2 headers
        .flag_if_supported("-std=c++17")
        .compile("rvo2_wrapper");

    // Link libraries
    println!("cargo:rustc-link-lib=ompl");
    println!("cargo:rustc-link-lib=RVO");
    
    // Add library search paths if needed
    println!("cargo:rustc-link-search=/usr/local/lib");
    
    println!("cargo:rerun-if-changed=src/ffi/ompl_bridge.rs");
    println!("cargo:rerun-if-changed=src/ffi/rvo2_bridge.rs");
    println!("cargo:rerun-if-changed=cpp/ompl_wrapper.cpp");
    println!("cargo:rerun-if-changed=cpp/rvo2_wrapper.cpp");
    println!("cargo:rerun-if-changed=cpp/ompl_wrapper.h");
    println!("cargo:rerun-if-changed=cpp/rvo2_wrapper.h");
}
```

## Part 2: Bevy Integration

### Rust Components and Resources

**src/main.rs:**
```rust
mod ffi;

use bevy::prelude::*;
use big_brain::prelude::*;
use cxx::UniquePtr;
use ffi::{ompl_ffi, rvo2_ffi};

// ============================================================================
// Components
// ============================================================================

#[derive(Component)]
struct Vehicle {
    max_speed: f32,
    acceleration: f32,
}

#[derive(Component)]
struct Physics {
    velocity: Vec3,
    acceleration: Vec3,
}

#[derive(Component)]
struct Destination {
    position: Vec3,
}

#[derive(Component)]
struct Path {
    waypoints: Vec<Vec3>,
    current_index: usize,
}

impl Path {
    fn current_waypoint(&self) -> Option<Vec3> {
        self.waypoints.get(self.current_index).copied()
    }
    
    fn advance(&mut self) {
        if self.current_index < self.waypoints.len() - 1 {
            self.current_index += 1;
        }
    }
    
    fn is_complete(&self) -> bool {
        self.current_index >= self.waypoints.len()
    }
}

#[derive(Component)]
struct RVOAgentId(usize);

#[derive(Component)]
struct NeedsPathPlanning;

#[derive(Component)]
struct DesiredVelocity(Vec3);

// ============================================================================
// Resources
// ============================================================================

#[derive(Resource)]
struct OMPLPlannerResource {
    planner: UniquePtr<ompl_ffi::OMPLPlanner>,
}

impl OMPLPlannerResource {
    fn new(bounds_min: Vec3, bounds_max: Vec3) -> Self {
        Self {
            planner: ompl_ffi::create_ompl_planner(
                bounds_min.into(),
                bounds_max.into()
            ),
        }
    }
}

#[derive(Resource)]
struct RVO2SimulatorResource {
    simulator: UniquePtr<rvo2_ffi::RVO2Simulator>,
}

impl RVO2SimulatorResource {
    fn new() -> Self {
        Self {
            simulator: rvo2_ffi::create_rvo2_simulator(),
        }
    }
}

// ============================================================================
// big_brain AI Components
// ============================================================================

#[derive(Component, Debug, Clone, ScorerBuilder)]
struct HasDestinationScorer;

#[derive(Component, Debug, Clone, ScorerBuilder)]
struct HasPathScorer;

#[derive(Component, Debug, Clone, ActionBuilder)]
struct PlanPathAction;

#[derive(Component, Debug, Clone, ActionBuilder)]
struct FollowPathAction;

#[derive(Component, Debug, Clone, ActionBuilder)]
struct IdleAction;

// ============================================================================
// Scorer Systems
// ============================================================================

fn has_destination_scorer(
    mut query: Query<(Entity, &mut Score, Option<&Destination>), With<HasDestinationScorer>>
) {
    for (entity, mut score, destination) in query.iter_mut() {
        let has_dest = destination.is_some();
        score.set(if has_dest { 1.0 } else { 0.0 });
    }
}

fn has_path_scorer(
    mut query: Query<(&mut Score, Option<&Path>), With<HasPathScorer>>
) {
    for (mut score, path) in query.iter_mut() {
        let has_valid_path = path.map(|p| !p.is_complete()).unwrap_or(false);
        score.set(if has_valid_path { 1.0 } else { 0.0 });
    }
}

// ============================================================================
// Action Systems
// ============================================================================

fn plan_path_action(
    mut commands: Commands,
    mut ompl: ResMut<OMPLPlannerResource>,
    mut query: Query<(
        Entity,
        &Transform,
        &Destination,
        &mut ActionState,
        Option<&Path>,
    ), With<PlanPathAction>>,
    obstacles: Query<(&Transform, &GlobalTransform), With<Obstacle>>,
) {
    for (entity, transform, destination, mut state, existing_path) in query.iter_mut() {
        match *state {
            ActionState::Requested => {
                info!("Entity {:?}: Planning path", entity);
                *state = ActionState::Executing;
            }
            ActionState::Executing => {
                // Update obstacles in OMPL
                ompl.planner.pin_mut().clear_obstacles();
                for (obs_transform, _) in obstacles.iter() {
                    ompl.planner.pin_mut().add_obstacle(
                        obs_transform.translation.into(),
                        5.0, // Obstacle radius
                    );
                }
                
                // Plan path
                let result = ompl.planner.pin_mut().plan_path(
                    transform.translation.into(),
                    destination.position.into(),
                    5.0, // 5 second timeout
                );
                
                if result.success && !result.waypoints.is_empty() {
                    info!("Entity {:?}: Path found with {} waypoints", entity, result.waypoints.len());
                    
                    let waypoints: Vec<Vec3> = result.waypoints
                        .into_iter()
                        .map(|w| w.into())
                        .collect();
                    
                    commands.entity(entity).insert(Path {
                        waypoints,
                        current_index: 0,
                    });
                    
                    *state = ActionState::Success;
                } else {
                    warn!("Entity {:?}: Path planning failed", entity);
                    *state = ActionState::Failure;
                }
            }
            ActionState::Cancelled => {
                *state = ActionState::Failure;
            }
            _ => {}
        }
    }
}

fn follow_path_action(
    mut query: Query<(
        Entity,
        &Transform,
        &mut Path,
        &mut DesiredVelocity,
        &mut ActionState,
        &Vehicle,
    ), With<FollowPathAction>>
) {
    for (entity, transform, mut path, mut desired_vel, mut state, vehicle) in query.iter_mut() {
        match *state {
            ActionState::Requested => {
                info!("Entity {:?}: Following path", entity);
                *state = ActionState::Executing;
            }
            ActionState::Executing => {
                if let Some(waypoint) = path.current_waypoint() {
                    let direction = (waypoint - transform.translation).normalize_or_zero();
                    let distance = transform.translation.distance(waypoint);
                    
                    // Set desired velocity toward waypoint
                    desired_vel.0 = direction * vehicle.max_speed;
                    
                    // Check if reached waypoint
                    if distance < 2.0 {
                        path.advance();
                        
                        if path.is_complete() {
                            info!("Entity {:?}: Path complete", entity);
                            *state = ActionState::Success;
                        }
                    }
                } else {
                    *state = ActionState::Success;
                }
            }
            ActionState::Cancelled => {
                desired_vel.0 = Vec3::ZERO;
                *state = ActionState::Failure;
            }
            _ => {}
        }
    }
}

fn idle_action(
    mut query: Query<(&mut DesiredVelocity, &mut ActionState), With<IdleAction>>
) {
    for (mut desired_vel, mut state) in query.iter_mut() {
        match *state {
            ActionState::Requested => {
                desired_vel.0 = Vec3::ZERO;
                *state = ActionState::Executing;
            }
            ActionState::Executing => {
                // Stay idle
                desired_vel.0 = Vec3::ZERO;
            }
            _ => {}
        }
    }
}

// ============================================================================
// RVO2 Collision Avoidance System
// ============================================================================

fn rvo2_collision_avoidance(
    mut rvo: ResMut<RVO2SimulatorResource>,
    mut query: Query<(
        &Transform,
        &mut Physics,
        &RVOAgentId,
        &DesiredVelocity,
    )>,
    time: Res<Time>,
) {
    // Update RVO2 simulator with current positions and desired velocities
    for (transform, physics, agent_id, desired_vel) in query.iter() {
        let pos_2d = rvo2_ffi::Vec2 {
            x: transform.translation.x,
            y: transform.translation.z,
        };
        
        let desired_vel_2d = rvo2_ffi::Vec2 {
            x: desired_vel.0.x,
            y: desired_vel.0.z,
        };
        
        rvo.simulator.pin_mut().set_agent_position(agent_id.0, pos_2d);
        rvo.simulator.pin_mut().set_agent_pref_velocity(agent_id.0, desired_vel_2d);
    }
    
    // Step RVO2 simulation
    rvo.simulator.pin_mut().do_step(time.delta_seconds());
    
    // Apply collision-free velocities back to entities
    for (transform, mut physics, agent_id, desired_vel) in query.iter_mut() {
        let safe_velocity = rvo.simulator.get_agent_velocity(agent_id.0);
        
        physics.velocity.x = safe_velocity.x;
        physics.velocity.z = safe_velocity.y;
        // Keep y velocity unchanged (vertical motion)
    }
}

// ============================================================================
// Physics System
// ============================================================================

fn physics_system(
    time: Res<Time>,
    mut query: Query<(&mut Transform, &mut Physics)>
) {
    for (mut transform, mut physics) in query.iter_mut() {
        // Apply acceleration
        physics.velocity += physics.acceleration * time.delta_seconds();
        
        // Apply position
        transform.translation += physics.velocity * time.delta_seconds();
        
        // Apply drag
        physics.velocity *= 0.98;
    }
}

// ============================================================================
// Setup and Spawning
// ============================================================================

#[derive(Component)]
struct Obstacle;

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut rvo: ResMut<RVO2SimulatorResource>,
) {
    // Camera
    commands.spawn(Camera3dBundle {
        transform: Transform::from_xyz(0.0, 50.0, 100.0)
            .looking_at(Vec3::ZERO, Vec3::Y),
        ..default()
    });
    
    // Light
    commands.spawn(DirectionalLightBundle {
        transform: Transform::from_rotation(Quat::from_euler(EulerRot::XYZ, -0.5, 0.5, 0.0)),
        ..default()
    });
    
    // Ground plane
    commands.spawn(PbrBundle {
        mesh: meshes.add(Plane3d::default().mesh().size(200.0, 200.0)),
        material: materials.add(Color::srgb(0.3, 0.5, 0.3)),
        ..default()
    });
    
    // Spawn obstacles
    for i in 0..5 {
        let x = (i as f32 - 2.0) * 20.0;
        commands.spawn((
            PbrBundle {
                mesh: meshes.add(Sphere::new(5.0)),
                material: materials.add(Color::srgb(0.8, 0.2, 0.2)),
                transform: Transform::from_xyz(x, 5.0, 0.0),
                ..default()
            },
            Obstacle,
        ));
    }
    
    // Spawn vehicles
    let vehicle_mesh = meshes.add(Cuboid::new(2.0, 1.0, 3.0));
    let vehicle_material = materials.add(Color::srgb(0.2, 0.4, 0.8));
    
    for i in 0..10 {
        let x = (i as f32 - 5.0) * 10.0;
        let start_pos = Vec3::new(x, 1.0, -40.0);
        
        // Add agent to RVO2
        let agent_id = rvo.simulator.pin_mut().add_agent(
            rvo2_ffi::Vec2 { x: start_pos.x, y: start_pos.z },
            rvo2_ffi::AgentParams {
                neighbor_dist: 15.0,
                max_neighbors: 10,
                time_horizon: 2.0,
                time_horizon_obst: 2.0,
                radius: 2.0,
                max_speed: 10.0,
                velocity: rvo2_ffi::Vec2 { x: 0.0, y: 0.0 },
            },
        );
        
        commands.spawn((
            PbrBundle {
                mesh: vehicle_mesh.clone(),
                material: vehicle_material.clone(),
                transform: Transform::from_translation(start_pos),
                ..default()
            },
            Vehicle {
                max_speed: 10.0,
                acceleration: 5.0,
            },
            Physics {
                velocity: Vec3::ZERO,
                acceleration: Vec3::ZERO,
            },
            Destination {
                position: Vec3::new(x, 1.0, 40.0),
            },
            DesiredVelocity(Vec3::ZERO),
            RVOAgentId(agent_id),
            // AI Thinker
            Thinker::build()
                .when(
                    HasDestinationScorer,
                    PlanPathAction
                )
                .when(
                    HasPathScorer,
                    FollowPathAction
                )
                .otherwise(IdleAction),
        ));
    }
}

// ============================================================================
// Main
// ============================================================================

fn main() {
    App::new()
        .add_plugins
(DefaultPlugins)
        .add_plugins(BigBrainPlugin::new(PreUpdate))
        // Resources
        .insert_resource(OMPLPlannerResource::new(
            Vec3::new(-100.0, 0.0, -100.0),
            Vec3::new(100.0, 20.0, 100.0),
        ))
        .insert_resource(RVO2SimulatorResource::new())
        // Setup
        .add_systems(Startup, setup)
        // Scorer systems
        .add_systems(Update, (
            has_destination_scorer,
            has_path_scorer,
        ))
        // Action systems
        .add_systems(Update, (
            plan_path_action,
            follow_path_action,
            idle_action,
        ))
        // Physics and collision avoidance
        .add_systems(Update, (
            rvo2_collision_avoidance,
            physics_system.after(rvo2_collision_avoidance),
        ))
        .run();
}
```

## Part 3: Decision Flow

Here's how the complete decision process works:

```
┌─────────────────────────────────────────────────────────────┐
│                      big_brain Thinker                       │
│                                                              │
│  ┌────────────────────────────────────────────────────┐    │
│  │  Scorers (evaluated every frame or throttled)      │    │
│  │                                                     │    │
│  │  • HasDestinationScorer: Do I have a goal?         │    │
│  │  • HasPathScorer: Do I have a valid path?          │    │
│  └────────────────────────────────────────────────────┘    │
│                           │                                  │
│                           ▼                                  │
│  ┌────────────────────────────────────────────────────┐    │
│  │  Action Selection (highest scoring action wins)    │    │
│  └────────────────────────────────────────────────────┘    │
│                           │                                  │
└───────────────────────────┼──────────────────────────────────┘
                            │
                            ▼
        ┌───────────────────────────────────────┐
        │  Selected Action Executes             │
        └───────────────────────────────────────┘
                            │
        ┌───────────────────┴───────────────────┐
        │                                        │
        ▼                                        ▼
┌──────────────────┐                   ┌──────────────────┐
│  PlanPathAction  │                   │ FollowPathAction │
│                  │                   │                  │
│  Calls OMPL:     │                   │  Reads Path      │
│  1. Clear obs    │                   │  2. Sets         │
│  2. Add obs      │                   │     Desired      │
│  3. Plan path    │                   │     Velocity     │
│  4. Store Path   │                   │  3. Advances     │
│     component    │                   │     waypoint     │
└──────────────────┘                   └──────────────────┘
        │                                        │
        │                                        │
        │              DesiredVelocity           │
        └────────────────────┬───────────────────┘
                             │
                             ▼
                ┌────────────────────────┐
                │ RVO2 Collision         │
                │ Avoidance System       │
                │                        │
                │ 1. Updates RVO2 with   │
                │    positions           │
                │ 2. Sets preferred vel  │
                │ 3. Steps simulation    │
                │ 4. Applies safe        │
                │    velocity to Physics │
                └────────────────────────┘
                             │
                             ▼
                ┌────────────────────────┐
                │   Physics System       │
                │                        │
                │ Integrates velocity    │
                │ Updates Transform      │
                └────────────────────────┘
```

## Part 4: Performance Optimization

### 1. Throttle Path Planning

```rust
#[derive(Component)]
struct PathPlanningTimer(Timer);

fn throttled_path_planning(
    time: Res<Time>,
    mut query: Query<(&mut PathPlanningTimer, &mut Thinker), With<NeedsPathPlanning>>
) {
    for (mut timer, mut thinker) in query.iter_mut() {
        timer.0.tick(time.delta());
        if timer.0.just_finished() {
            // Force re-evaluation of path planning
            thinker.force_update();
        }
    }
}
```

### 2. Async Path Planning

```rust
use bevy::tasks::{AsyncComputeTaskPool, Task};

#[derive(Component)]
struct PathPlanningTask(Task<Option<Vec<Vec3>>>);

fn spawn_async_planning(
    mut commands: Commands,
    ompl: Res<OMPLPlannerResource>,
    query: Query<(Entity, &Transform, &Destination), With<NeedsPathPlanning>>
) {
    let thread_pool = AsyncComputeTaskPool::get();
    
    for (entity, transform, destination) in query.iter() {
        let start = transform.translation;
        let goal = destination.position;
        
        // Spawn planning task
        let task = thread_pool.spawn(async move {
            // Planning happens on background thread
            // Note: Need thread-safe wrapper for OMPL
            None // Simplified
        });
        
        commands.entity(entity).insert(PathPlanningTask(task));
    }
}
```

### 3. Spatial Partitioning for RVO2

Only update nearby agents in RVO2:

```rust
fn optimized_rvo2(
    mut rvo: ResMut<RVO2SimulatorResource>,
    query: Query<(&Transform, &RVOAgentId)>,
    active_agents: Query<Entity, With<ActiveAgent>>,
) {
    // Only update active agents
    for entity in active_agents.iter() {
        if let Ok((transform, agent_id)) = query.get(entity) {
            // Update only active agents
        }
    }
}
```

## Summary

**Integration Strategy:**
1. ✅ OMPL for global path planning (every few seconds)
2. ✅ big_brain for high-level decision making (when to plan, when to follow)
3. ✅ RVO2 for local collision avoidance (every frame)
4. ✅ All integrated through Bevy ECS components

**Performance:**
- OMPL: ~5-50ms per planning query (infrequent)
- RVO2: ~0.5-2ms per frame for 100 agents (every frame)
- FFI overhead: <1% of total computation

**Advantages:**
- Proven, battle-tested algorithms
- Separation of concerns (global vs local planning)
- Full integration with Bevy ECS and big_brain
- Type-safe Rust/C++ boundary with `cxx`

This architecture gives you sophisticated autonomous vehicle behavior with global path planning avoiding static obstacles (OMPL) and local collision avoidance with dynamic agents (RVO2), all orchestrated through big_brain's decision system.

---

# RVO2 and Nonholonomic Vehicles

**Short answer: No, RVO2 does not natively support nonholonomic constraints.**

RVO2 (and ORCA - Optimal Reciprocal Collision Avoidance) assumes **holonomic agents** - agents that can move instantly in any direction. This is a significant limitation for car-like vehicles.

## The Problem with RVO2 for Vehicles

### What RVO2 Assumes

```rust
// RVO2 computes a velocity that can be applied instantly
let safe_velocity = rvo.simulator.get_agent_velocity(agent_id);

// For a holonomic agent (person, drone):
agent.velocity = safe_velocity; // ✅ Works fine

// For a car:
agent.velocity = safe_velocity; // ❌ Violates physics!
// Cars can't instantly change direction or move sideways
```

### Nonholonomic Constraints for Cars

Car-like vehicles have constraints:
- **Cannot move sideways** (no lateral velocity)
- **Limited turning radius** (minimum turn radius based on wheelbase)
- **Steering rate limits** (can't turn steering wheel infinitely fast)
- **Forward/backward motion only** along heading direction

```rust
// A car's motion is constrained:
struct CarConstraints {
    max_steering_angle: f32,    // e.g., ±30°
    wheelbase: f32,              // Distance between axles
    max_steering_rate: f32,      // How fast steering can change
}

// Valid car motion:
velocity = forward_speed * heading_direction; // Only moves along heading

// Invalid (but RVO2 might suggest this):
velocity = Vec3::new(vx, 0, vz); // Arbitrary direction
```

## Alternative Solutions

### Option 1: NH-ORCA (Nonholonomic ORCA)

**NH-ORCA** is an extension of ORCA specifically for nonholonomic vehicles, but:
- ❌ No mature C++ library available
- ❌ Research code only (not production-ready)
- ❌ Limited to specific vehicle models

### Option 2: Use RVO2 with Post-Processing (Hybrid Approach)

This is the most practical solution - use RVO2 for collision avoidance intent, then project onto feasible vehicle motion:

```rust
#[derive(Component)]
struct CarDynamics {
    wheelbase: f32,
    max_steering_angle: f32,
    max_steering_rate: f32,
    current_steering_angle: f32,
    max_speed: f32,
    max_acceleration: f32,
}

fn rvo2_with_car_constraints(
    mut rvo: ResMut<RVO2SimulatorResource>,
    mut query: Query<(
        &Transform,
        &mut Physics,
        &RVOAgentId,
        &DesiredVelocity,
        &mut CarDynamics,
    )>,
    time: Res<Time>,
) {
    // Step 1: Update RVO2 with desired velocities
    for (transform, physics, agent_id, desired_vel, car) in query.iter() {
        let pos_2d = rvo2_ffi::Vec2 {
            x: transform.translation.x,
            y: transform.translation.z,
        };
        
        let desired_vel_2d = rvo2_ffi::Vec2 {
            x: desired_vel.0.x,
            y: desired_vel.0.z,
        };
        
        rvo.simulator.pin_mut().set_agent_position(agent_id.0, pos_2d);
        rvo.simulator.pin_mut().set_agent_pref_velocity(agent_id.0, desired_vel_2d);
    }
    
    // Step 2: Run RVO2
    rvo.simulator.pin_mut().do_step(time.delta_seconds());
    
    // Step 3: Project RVO2 velocities onto feasible car motion
    for (transform, mut physics, agent_id, desired_vel, mut car) in query.iter_mut() {
        let rvo_velocity = rvo.simulator.get_agent_velocity(agent_id.0);
        let target_velocity = Vec3::new(rvo_velocity.x, 0.0, rvo_velocity.y);
        
        // Get current heading
        let heading = transform.forward();
        let heading_2d = Vec3::new(heading.x, 0.0, heading.z).normalize();
        
        // Project target velocity onto feasible motion
        let feasible_velocity = project_to_car_motion(
            target_velocity,
            heading_2d,
            &mut car,
            time.delta_seconds(),
        );
        
        physics.velocity = feasible_velocity;
    }
}

fn project_to_car_motion(
    target_velocity: Vec3,
    current_heading: Vec3,
    car: &mut CarDynamics,
    dt: f32,
) -> Vec3 {
    if target_velocity.length() < 0.1 {
        return Vec3::ZERO;
    }
    
    let target_direction = target_velocity.normalize();
    let target_speed = target_velocity.length().min(car.max_speed);
    
    // Calculate desired steering angle
    // Angle between current heading and target direction
    let cross = current_heading.cross(target_direction).y;
    let dot = current_heading.dot(target_direction);
    let angle_to_target = cross.atan2(dot);
    
    // Clamp to maximum steering angle
    let desired_steering = angle_to_target.clamp(
        -car.max_steering_angle,
        car.max_steering_angle
    );
    
    // Apply steering rate limit
    let steering_change = (desired_steering - car.current_steering_angle)
        .clamp(-car.max_steering_rate * dt, car.max_steering_rate * dt);
    
    car.current_steering_angle += steering_change;
    car.current_steering_angle = car.current_steering_angle.clamp(
        -car.max_steering_angle,
        car.max_steering_angle
    );
    
    // Calculate actual velocity based on current heading and steering
    // Use bicycle model approximation
    let turning_radius = if car.current_steering_angle.abs() > 0.01 {
        car.wheelbase / car.current_steering_angle.tan()
    } else {
        f32::INFINITY
    };
    
    // For simplicity, move along current heading
    // More sophisticated: compute actual turning motion
    let forward_speed = target_speed * dot.max(0.0); // Reduce speed when turning away
    
    current_heading * forward_speed
}
```

### Option 3: Model Predictive Control (MPC) with RVO2

Use RVO2 to define a collision-free region, then use MPC to find feasible trajectory:

```rust
#[derive(Component)]
struct MPCController {
    horizon: usize,        // Planning horizon (e.g., 20 steps)
    dt: f32,               // Time step
    state_buffer: Vec<CarState>,
}

#[derive(Clone)]
struct CarState {
    position: Vec3,
    heading: f32,
    velocity: f32,
    steering_angle: f32,
}

fn mpc_with_rvo2_constraints(
    mut rvo: ResMut<RVO2SimulatorResource>,
    mut query: Query<(
        &Transform,
        &mut Physics,
        &RVOAgentId,
        &DesiredVelocity,
        &CarDynamics,
        &mut MPCController,
    )>,
    time: Res<Time>,
) {
    // Get RVO2 safe velocities (defines collision-free regions)
    rvo.simulator.pin_mut().do_step(time.delta_seconds());
    
    for (transform, mut physics, agent_id, desired_vel, car, mut mpc) in query.iter_mut() {
        let rvo_velocity = rvo.simulator.get_agent_velocity(agent_id.0);
        
        // Solve MPC optimization:
        // Minimize: distance to RVO velocity
        // Subject to: car dynamics, steering limits, collision avoidance
        let optimal_control = solve_mpc(
            transform,
            car,
            Vec3::new(rvo_velocity.x, 0.0, rvo_velocity.y),
            &mpc,
        );
        
        // Apply first control action
        physics.velocity = optimal_control.velocity;
    }
}

fn solve_mpc(
    transform: &Transform,
    car: &CarDynamics,
    target_velocity: Vec3,
    mpc: &MPCController,
) -> ControlAction {
    // Simplified: Should use optimization library
    // For production: use optimization crate like `argmin` or call C++ optimizer
    
    ControlAction {
        velocity: target_velocity, // Placeholder
        steering: 0.0,
    }
}

struct ControlAction {
    velocity: Vec3,
    steering: f32,
}
```

### Option 4: Replace RVO2 with Vehicle-Specific Algorithm

Use algorithms designed for vehicles:

#### A. Velocity Obstacles for Nonholonomic Vehicles

```rust
// Custom implementation respecting car dynamics
fn velocity_obstacles_cars(
    mut query: Query<(
        &Transform,
        &mut Physics,
        &DesiredVelocity,
        &CarDynamics,
    )>,
    other_vehicles: Query<(&Transform, &Physics), Without<DesiredVelocity>>,
) {
    for (transform, mut physics, desired_vel, car) in query.iter_mut() {
        let mut best_velocity = desired_vel.0;
        let mut min_cost = f32::MAX;
        
        // Sample feasible velocities (reachable by car)
        for sample in sample_feasible_velocities(transform, car) {
            let mut cost = (sample - desired_vel.0).length();
            
            // Check collision with other vehicles
            let mut is_safe = true;
            for (other_transform, other_physics) in other_vehicles.iter() {
                if will_collide(
                    transform.translation,
                    sample,
                    other_transform.translation,
                    other_physics.velocity,
                    2.0, // Time horizon
                ) {
                    is_safe = false;
                    break;
                }
            }
            
            if is_safe && cost < min_cost {
                min_cost = cost;
                best_velocity = sample;
            }
        }
        
        physics.velocity = best_velocity;
    }
}

fn sample_feasible_velocities(
    transform: &Transform,
    car: &CarDynamics,
) -> Vec<Vec3> {
    let mut samples = Vec::new();
    let heading = transform.forward();
    
    // Sample different speeds and steering angles
    for speed in 0..10 {
        let speed_val = speed as f32 * car.max_speed / 10.0;
        
        for steering_idx in -5..=5 {
            let steering = (steering_idx as f32 / 5.0) * car.max_steering_angle;
            
            // Compute velocity for this steering angle
            // Simplified: should use bicycle model
            let turn_angle = steering * 0.1; // Approximate
            let direction = Quat::from_rotation_y(turn_angle) * heading;
            
            samples.push(direction * speed_val);
        }
    }
    
    samples
}

fn will_collide(
    pos1: Vec3,
    vel1: Vec3,
    pos2: Vec3,
    vel2: Vec3,
    time_horizon: f32,
) -> bool {
    // Check if trajectories will collide within time_horizon
    let relative_pos = pos1 - pos2;
    let relative_vel = vel1 - vel2;
    
    // Time of closest approach
    let t = -(relative_pos.dot(relative_vel)) / relative_vel.length_squared();
    
    if t < 0.0 || t > time_horizon {
        return false;
    }
    
    let closest_distance = (relative_pos + relative_vel * t).length();
    closest_distance < 4.0 // Safety radius
}
```

#### B. Social Force Model (Better for vehicles)

```rust
fn social_force_model(
    mut query: Query<(&Transform, &mut Physics, &DesiredVelocity, &CarDynamics)>,
    other_vehicles: Query<(&Transform, &Physics), Without<DesiredVelocity>>,
) {
    for (transform, mut physics, desired_vel, car) in query.iter_mut() {
        let mut force = Vec3::ZERO;
        
        // Attraction to goal
        let goal_force = (desired_vel.0 - physics.velocity) * 2.0;
        force += goal_force;
        
        // Repulsion from other vehicles
        for (other_transform, other_physics) in other_vehicles.iter() {
            let diff = transform.translation - other_transform.translation;
            let distance = diff.length();
            
            if distance < 20.0 {
                let repulsion_force = diff.normalize() * (100.0 / (distance * distance));
                force += repulsion_force;
            }
        }
        
        // Project force onto feasible direction (heading)
        let heading = transform.forward();
        let forward_force = force.dot(heading);
        
        // Apply only forward component (cars can't move sideways)
        physics.acceleration = heading * forward_force.max(0.0);
    }
}
```

### Option 5: Hybrid OMPL + Local Tracking

Skip RVO2 entirely and use pure path following with local obstacle avoidance:

```rust
fn dynamic_window_approach(
    mut query: Query<(
        &Transform,
        &mut Physics,
        &Path,
        &CarDynamics,
    )>,
    obstacles: Query<&Transform, With<Obstacle>>,
) {
    for (transform, mut physics, path, car) in query.iter_mut() {
        if let Some(target) = path.current_waypoint() {
            let mut best_velocity = Vec3::ZERO;
            let mut best_score = f32::MIN;
            
            // Sample velocities within dynamic window (reachable in next time step)
            for sample in sample_dynamic_window(physics.velocity, car) {
                let score = evaluate_trajectory(
                    transform,
                    sample,
                    target,
                    &obstacles,
                );
                
                if score > best_score {
                    best_score = score;
                    best_velocity = sample;
                }
            }
            
            physics.velocity = best_velocity;
        }
    }
}

fn sample_dynamic_window(
    current_velocity: Vec3,
    car: &CarDynamics,
) -> Vec<Vec3> {
    // Sample velocities reachable within one time step
    // considering acceleration and steering limits
    vec![] // Simplified
}

fn evaluate_trajectory(
    transform: &Transform,
    velocity: Vec3,
    target: Vec3,
    obstacles: &Query<&Transform, With<Obstacle>>,
) -> f32 {
    let mut score = 0.0;
    
    // Heading toward target
    let to_target = (target - transform.translation).normalize();
    score += velocity.normalize().dot(to_target) * 10.0;
    
    // Speed
    score += velocity.length();
    
    // Clearance from obstacles
    let mut min_clearance = f32::MAX;
    for obstacle in obstacles.iter() {
        let distance = transform.translation.distance(obstacle.translation);
        min_clearance = min_clearance.min(distance);
    }
    score += min_clearance;
    
    score
}
```

## My Recommendation for Vehicle Fleet

**Use a Hybrid Approach:**

1. **OMPL for global planning** (static obstacles) ✅
2. **Skip RVO2** or use it only as a hint ❌
3. **Implement vehicle-specific local avoidance:**

```rust
// Recommended architecture
fn vehicle_collision_avoidance(
    mut query: Query<(
        &Transform,
        &mut Physics,
        &Path,
        &DesiredVelocity,
        &mut CarDynamics,
    )>,
    other_vehicles: Query<(&Transform, &Physics), Without<Path>>,
    time: Res<Time>,
) {
    for (transform, mut physics, path, desired_vel, mut car) in query.iter_mut() {
        // 1. Get desired velocity from path following
        let path_velocity = compute_path_following_velocity(transform, path, car);
        
        // 2. Check for conflicts with other vehicles
        let mut adjusted_velocity = path_velocity;
        
        for (other_transform, other_physics) in other_vehicles.iter() {
            let collision_time = predict_collision(
                transform.translation,
                path_velocity,
                other_transform.translation,
                other_physics.velocity,
            );
            
            if let Some(t) = collision_time {
                if t < 3.0 { // 3 second horizon
                    // Slow down or adjust to avoid
                    adjusted_velocity = compute_avoidance_velocity(
                        transform,
                        path_velocity,
                        other_transform.translation,
                        other_physics.velocity,
                        car,
                    );
                }
            }
        }
        
        // 3. Apply car dynamics constraints
        let feasible_velocity = apply_car_constraints(
            adjusted_velocity,
            physics.velocity,
            transform,
            car,
            time.delta_seconds(),
        );
        
        physics.velocity = feasible_velocity;
    }
}
```

**Summary:**
- ❌ RVO2 is **not suitable** for car-like vehicles without heavy modification
- ✅ Use **hybrid approaches** that respect vehicle dynamics
- ✅ Consider **Dynamic Window Approach** or **Model Predictive Control**
- ✅ Keep OMPL for global planning - it works great
- ✅ Implement custom local avoidance that respects nonholonomic constraints

For production vehicle simulation, you'll get better results with vehicle-aware algorithms than trying to force RVO2 to work with cars.

---

# Identifying Specific Entities in Bevy ECS

There are several approaches to identify and track specific entities in Bevy ECS, each with different trade-offs.

## Method 1: Using Entity ID (Built-in)

**Best for**: Temporary references, parent-child relationships

Every entity has a unique `Entity` ID that Bevy manages:

```rust
use bevy::prelude::*;

fn spawn_vehicles(mut commands: Commands) {
    // Spawn and store entity IDs
    let vehicle1 = commands.spawn((
        Transform::default(),
        Vehicle { name: "Car 1".to_string() },
    )).id();
    
    let vehicle2 = commands.spawn((
        Transform::default(),
        Vehicle { name: "Car 2".to_string() },
    )).id();
    
    println!("Vehicle 1 Entity: {:?}", vehicle1); // Entity { index: 0, generation: 1 }
    println!("Vehicle 2 Entity: {:?}", vehicle2); // Entity { index: 1, generation: 1 }
}

fn control_specific_vehicle(
    keyboard: Res<ButtonInput<KeyCode>>,
    mut query: Query<(&mut Transform, &Vehicle)>,
    player_vehicle: Res<PlayerVehicleEntity>, // Store specific entity
) {
    if let Ok((mut transform, vehicle)) = query.get_mut(player_vehicle.0) {
        if keyboard.pressed(KeyCode::KeyW) {
            transform.translation.z += 1.0;
        }
    }
}

#[derive(Resource)]
struct PlayerVehicleEntity(Entity);
```

**Pros:**
- No memory overhead
- Automatically handled by Bevy
- Safe (generation prevents use-after-free)

**Cons:**
- Not human-readable
- Can't serialize/deserialize easily
- Lost if you despawn and respawn

## Method 2: Custom ID Component

**Best for**: Persistent identification, save/load systems, networking

Create your own ID component:

```rust
use bevy::prelude::*;

#[derive(Component, Debug, Clone, Copy, PartialEq, Eq, Hash)]
struct VehicleId(u32);

#[derive(Component)]
struct Vehicle {
    max_speed: f32,
}

fn spawn_fleet(mut commands: Commands) {
    // Spawn vehicles with custom IDs
    for i in 0..100 {
        commands.spawn((
            VehicleId(i),
            Transform::from_xyz(i as f32 * 5.0, 0.0, 0.0),
            Vehicle { max_speed: 10.0 },
        ));
    }
}

// Find specific vehicle by ID
fn find_vehicle_by_id(
    query: Query<(Entity, &VehicleId, &Transform)>,
    target_id: u32,
) -> Option<(Entity, Vec3)> {
    query.iter()
        .find(|(_, id, _)| id.0 == target_id)
        .map(|(entity, _, transform)| (entity, transform.translation))
}

// Control specific vehicle
fn control_vehicle_by_id(
    mut query: Query<(&VehicleId, &mut Transform)>,
    keyboard: Res<ButtonInput<KeyCode>>,
) {
    // Control vehicle with ID 42
    for (id, mut transform) in query.iter_mut() {
        if id.0 == 42 {
            if keyboard.pressed(KeyCode::KeyW) {
                transform.translation.z += 1.0;
            }
        }
    }
}
```

**Better: Use a lookup table for performance**

```rust
use std::collections::HashMap;

#[derive(Resource, Default)]
struct VehicleRegistry {
    id_to_entity: HashMap<u32, Entity>,
}

fn spawn_registered_vehicles(
    mut commands: Commands,
    mut registry: ResMut<VehicleRegistry>,
) {
    for i in 0..100 {
        let entity = commands.spawn((
            VehicleId(i),
            Transform::default(),
            Vehicle { max_speed: 10.0 },
        )).id();
        
        registry.id_to_entity.insert(i, entity);
    }
}

fn control_by_id_fast(
    registry: Res<VehicleRegistry>,
    mut query: Query<&mut Transform>,
    keyboard: Res<ButtonInput<KeyCode>>,
) {
    // O(1) lookup instead of O(n) iteration
    if let Some(&entity) = registry.id_to_entity.get(&42) {
        if let Ok(mut transform) = query.get_mut(entity) {
            if keyboard.pressed(KeyCode::KeyW) {
                transform.translation.z += 1.0;
            }
        }
    }
}

// Clean up registry when entities are despawned
fn cleanup_registry(
    mut removed: RemovedComponents<VehicleId>,
    mut registry: ResMut<VehicleRegistry>,
    query: Query<&VehicleId>,
) {
    for entity in removed.read() {
        // Entity was despawned, find its ID
        registry.id_to_entity.retain(|_, &mut e| e != entity);
    }
}
```

## Method 3: Name/String Identifier

**Best for**: Human-readable identification, debugging, editor integration

```rust
use bevy::prelude::*;

#[derive(Component, Debug, Clone)]
struct VehicleName(String);

fn spawn_named_vehicles(mut commands: Commands) {
    commands.spawn((
        VehicleName("Player Car".to_string()),
        Transform::default(),
        Vehicle { max_speed: 15.0 },
    ));
    
    commands.spawn((
        VehicleName("AI Car 1".to_string()),
        Transform::default(),
        Vehicle { max_speed: 10.0 },
    ));
    
    commands.spawn((
        VehicleName("Police Car".to_string()),
        Transform::default(),
        Vehicle { max_speed: 12.0 },
    ));
}

fn find_by_name(
    query: Query<(Entity, &VehicleName, &Transform)>,
) {
    for (entity, name, transform) in query.iter() {
        if name.0 == "Player Car" {
            println!("Player car at: {:?}", transform.translation);
        }
    }
}

// With lookup table for better performance
#[derive(Resource, Default)]
struct VehicleNameRegistry {
    name_to_entity: HashMap<String, Entity>,
}

fn spawn_with_name_registry(
    mut commands: Commands,
    mut registry: ResMut<VehicleNameRegistry>,
) {
    let entity = commands.spawn((
        VehicleName("Player Car".to_string()),
        Transform::default(),
        Vehicle { max_speed: 15.0 },
    )).id();
    
    registry.name_to_entity.insert("Player Car".to_string(), entity);
}
```

## Method 4: Marker Components (Type-based)

**Best for**: Categories of entities, role-based identification

```rust
use bevy::prelude::*;

#[derive(Component)]
struct PlayerVehicle;

#[derive(Component)]
struct AIVehicle {
    aggression: f32,
}

#[derive(Component)]
struct EmergencyVehicle {
    siren_active: bool,
}

fn spawn_different_types(mut commands: Commands) {
    // Player vehicle
    commands.spawn((
        PlayerVehicle,
        Transform::default(),
        Vehicle { max_speed: 15.0 },
    ));
    
    // AI vehicles
    for i in 0..50 {
        commands.spawn((
            AIVehicle { aggression: 0.5 },
            Transform::from_xyz(i as f32 * 10.0, 0.0, 0.0),
            Vehicle { max_speed: 10.0 },
        ));
    }
    
    // Emergency vehicle
    commands.spawn((
        EmergencyVehicle { siren_active: false },
        Transform::default(),
        Vehicle { max_speed: 12.0 },
    ));
}

// Query by type
fn control_player(
    mut query: Query<&mut Transform, With<PlayerVehicle>>,
    keyboard: Res<ButtonInput<KeyCode>>,
) {
    // Only one player vehicle, so single() is safe
    if let Ok(mut transform) = query.get_single_mut() {
        if keyboard.pressed(KeyCode::KeyW) {
            transform.translation.z += 1.0;
        }
    }
}

fn ai_behavior(
    mut query: Query<(&mut Transform, &AIVehicle)>,
) {
    for (mut transform, ai) in query.iter_mut() {
        // AI logic based on aggression
        transform.translation.x += ai.aggression;
    }
}

fn emergency_behavior(
    mut query: Query<(&mut Transform, &mut EmergencyVehicle)>,
) {
    for (mut transform, mut emergency) in query.iter_mut() {
        if emergency.siren_active {
            // Emergency behavior
            transform.translation.z += 2.0;
        }
    }
}
```

## Method 5: Hierarchical Identification (Parent-Child)

**Best for**: Complex vehicles with sub-components

```rust
use bevy::prelude::*;

fn spawn_complex_vehicle(mut commands: Commands) {
    // Parent vehicle
    commands.spawn((
        VehicleId(1),
        Transform::default(),
        Vehicle { max_speed: 10.0 },
    ))
    .with_children(|parent| {
        // Wheels
        parent.spawn((
            WheelId(0), // Front-left
            Transform::from_xyz(-1.0, 0.0, 1.5),
            Wheel { steering_angle: 0.0 },
        ));
        
        parent.spawn((
            WheelId(1), // Front-right
            Transform::from_xyz(1.0, 0.0, 1.5),
            Wheel { steering_angle: 0.0 },
        ));
        
        parent.spawn((
            WheelId(2), // Rear-left
            Transform::from_xyz(-1.0, 0.0, -1.5),
            Wheel { steering_angle: 0.0 },
        ));
        
        parent.spawn((
            WheelId(3), // Rear-right
            Transform::from_xyz(1.0, 0.0, -1.5),
            Wheel { steering_angle: 0.0 },
        ));
    });
}

#[derive(Component)]
struct WheelId(u32);

#[derive(Component)]
struct Wheel {
    steering_angle: f32,
}

// Access child components
fn steer_front_wheels(
    vehicle_query: Query<(&VehicleId, &Children)>,
    mut wheel_query: Query<(&WheelId, &mut Wheel)>,
) {
    // Find vehicle 1
    for (vehicle_id, children) in vehicle_query.iter() {
        if vehicle_id.0 == 1 {
            // Access its wheels
            for &child in children.iter() {
                if let Ok((wheel_id, mut wheel)) = wheel_query.get_mut(child) {
                    // Steer front wheels only (IDs 0 and 1)
                    if wheel_id.0 < 2 {
                        wheel.steering_angle = 0.5;
                    }
                }
            }
        }
    }
}

// Or use Parent component to go up the hierarchy
fn wheel_knows_vehicle(
    wheel_query: Query<(&WheelId, &Parent), With<Wheel>>,
    vehicle_query: Query<&VehicleId>,
) {
    for (wheel_id, parent) in wheel_query.iter() {
        if let Ok(vehicle_id) = vehicle_query.get(parent.get()) {
            println!("Wheel {} belongs to vehicle {}", wheel_id.0, vehicle_id.0);
        }
    }
}
```

## Method 6: UUID for Globally Unique IDs

**Best for**: Networking, distributed systems, avoiding ID collisions

```rust
use bevy::prelude::*;
use uuid::Uuid;

#[derive(Component, Debug, Clone, Copy, PartialEq, Eq, Hash)]
struct VehicleUuid(Uuid);

fn spawn_with_uuid(mut commands: Commands) {
    let uuid = Uuid::new_v4();
    
    commands.spawn((
        VehicleUuid(uuid),
        Transform::default(),
        Vehicle { max_speed: 10.0 },
    ));
    
    println!("Spawned vehicle with UUID: {}", uuid);
}

#[derive(Resource, Default)]
struct VehicleUuidRegistry {
    uuid_to_entity: HashMap<Uuid, Entity>,
}

fn register_uuid(
    mut registry: ResMut<VehicleUuidRegistry>,
    query: Query<(Entity, &VehicleUuid), Added<VehicleUuid>>,
) {
    for (entity, uuid) in query.iter() {
        registry.uuid_to_entity.insert(uuid.0, entity);
    }
}
```

## Comprehensive Example: Fleet Management System

Here's a complete example combining multiple identification methods:

```rust
use bevy::prelude::*;
use std::collections::HashMap;

// ============================================================================
// Components
// ============================================================================

#[derive(Component, Debug, Clone, Copy, PartialEq, Eq, Hash)]
struct VehicleId(u32);

#[derive(Component, Debug, Clone)]
struct VehicleName(String);

#[derive(Component, Debug, Clone, Copy, PartialEq, Eq)]
enum VehicleType {
    Sedan,
    Truck,
    Bus,
    Emergency,
}

#[derive(Component)]
struct Vehicle {
    max_speed: f32,
    current_speed: f32,
}

#[derive(Component)]
struct PlayerControlled;

// ============================================================================
// Resources
// ============================================================================

#[derive(Resource, Default)]
struct VehicleRegistry {
    id_to_entity: HashMap<u32, Entity>,
    name_to_entity: HashMap<String, Entity>,
    entity_to_id: HashMap<Entity, u32>,
}

impl VehicleRegistry {
    fn register(&mut self, id: u32, name: String, entity: Entity) {
        self.id_to_entity.insert(id, entity);
        self.name_to_entity.insert(name, entity);
        self.entity_to_id.insert(entity, id);
    }
    
    fn get_by_id(&self, id: u32) -> Option<Entity> {
        self.id_to_entity.get(&id).copied()
    }
    
    fn get_by_name(&self, name: &str) -> Option<Entity> {
        self.name_to_entity.get(name).copied()
    }
    
    fn get_id(&self, entity: Entity) -> Option<u32> {
        self.entity_to_id.get(&entity).copied()
    }
    
    fn unregister(&mut self, entity: Entity) {
        if let Some(id) = self.entity_to_id.remove(&entity) {
            self.id_to_entity.remove(&id);
            // Remove from name_to_entity (requires iterating)
            self.name_to_entity.retain(|_, &mut e| e != entity);
        }
    }
}

// ============================================================================
// Spawning System
// ============================================================================

fn spawn_fleet(
    mut commands: Commands,
    mut registry: ResMut<VehicleRegistry>,
) {
    // Player vehicle
    let player_entity = commands.spawn((
        VehicleId(0),
        VehicleName("Player".to_string()),
        VehicleType::Sedan,
        PlayerControlled,
        Transform::from_xyz(0.0, 0.0, 0.0),
        Vehicle {
            max_speed: 15.0,
            current_speed: 0.0,
        },
    )).id();
    
    registry.register(0, "Player".to_string(), player_entity);
    
    // AI vehicles
    for i in 1..=50 {
        let vehicle_type = match i % 4 {
            0 => VehicleType::Sedan,
            1 => VehicleType::Truck,
            2 => VehicleType::Bus,
            _ => VehicleType::Emergency,
        };
        
        let name = format!("AI_{}", i);
        
        let entity = commands.spawn((
            VehicleId(i),
            VehicleName(name.clone()),
            vehicle_type,
            Transform::from_xyz(i as f32 * 10.0, 0.0, 0.0),
            Vehicle {
                max_speed: 10.0,
                current_speed: 0.0,
            },
        )).id();
        
        registry.register(i, name, entity);
    }
    
    info!("Spawned {} vehicles", registry.id_to_entity.len());
}

// ============================================================================
// Query Systems
// ============================================================================

fn find_vehicles_examples(
    registry: Res<VehicleRegistry>,
    query: Query<(&VehicleId, &VehicleName, &Transform)>,
    player_query: Query<&Transform, With<PlayerControlled>>,
    emergency_query: Query<(&VehicleId, &Transform), With<VehicleType>>,
) {
    // Method 1: Find by ID using registry (O(1))
    if let Some(entity) = registry.get_by_id(5) {
        if let Ok((id, name, transform)) = query.get(entity) {
            info!("Vehicle {}: {} at {:?}", id.0, name.0, transform.translation);
        }
    }
    
    // Method 2: Find by name using registry (O(1))
    if let Some(entity) = registry.get_by_name("AI_10") {
        if let Ok((id, name, transform)) = query.get(entity) {
            info!("Found by name: Vehicle {}: {}", id.0, name.0);
        }
    }
    
    // Method 3: Find player (marker component)
    if let Ok(transform) = player_query.get_single() {
        info!("Player at: {:?}", transform.translation);
    }
    
    // Method 4: Find by type (component filter)
    for (id, transform) in emergency_query.iter() {
        info!("Emergency vehicle {} at {:?}", id.0, transform.translation);
    }
    
    // Method 5: Find all vehicles in area
    let center = Vec3::ZERO;
    let radius = 50.0;
    
    for (id, name, transform) in query.iter() {
        if transform.translation.distance(center) < radius {
            info!("Vehicle {} in radius: {}", id.0, name.0);
        }
    }
}

// ============================================================================
// Control Systems
// ============================================================================

fn control_specific_vehicles(
    registry: Res<VehicleRegistry>,
    mut query: Query<(&VehicleId, &mut Vehicle, &mut Transform)>,
    keyboard: Res<ButtonInput<KeyCode>>,
) {
    // Control vehicle by ID
    if keyboard.just_pressed(KeyCode::Digit1) {
        if let Some(entity) = registry.get_by_id(1) {
            if let Ok((_, mut vehicle, mut transform)) = query.get_mut(entity) {
                vehicle.current_speed = vehicle.max_speed;
                info!("Accelerating vehicle 1");
            }
        }
    }
    
    // Control vehicle by name
    if keyboard.just_pressed(KeyCode::Digit2) {
        if let Some(entity) = registry.get_by_name("AI_5") {
            if let Ok((_, mut vehicle, mut transform)) = query.get_mut(entity) {
                transform.translation.y += 5.0; // Jump!
                info!("AI_5 jumped");
            }
        }
    }
}

// ============================================================================
// Cleanup System
// ============================================================================

fn cleanup_despawned_vehicles(
    mut removed: RemovedComponents<VehicleId>,
    mut registry: ResMut<VehicleRegistry>,
) {
    for entity in removed.read() {
        registry.unregister(entity);
        info!("Unregistered vehicle entity: {:?}", entity);
    }
}

// ============================================================================
// Main
// ============================================================================

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .init_resource::<VehicleRegistry>()
        .add_systems(Startup, spawn_fleet)
        .add_systems(Update, (
            find_vehicles_examples,
            control_specific_vehicles,
            cleanup_despawned_vehicles,
        ))
        .run();
}
```

## Performance Comparison

| Method | Lookup Speed | Memory | Use Case |
|--------|--------------|--------|----------|
| Entity ID | O(1) | None | Temporary references |
| Custom ID + HashMap | O(1) | Small | Persistent ID, fast lookup |
| Name + HashMap | O(1) | Medium | Human-readable, debugging |
| Marker Component | O(n) | None | Type-based queries |
| Linear search | O(n) | None | Small sets only |

## My Recommendation

**Use a combination:**

1. **`VehicleId` component** for persistent identification
2. **Registry resource** with HashMap for O(1) lookups
3. **Marker components** (`PlayerControlled`, `AIVehicle`) for type-based queries
4. **`VehicleName`** for debugging and human-readable logs

This gives you the best of all worlds: fast lookups, clear code, and flexible querying.

---

In Bevy, every entity is automatically assigned a unique `Entity` ID when spawned. However, these IDs are **ephemeral**—they can be reused by the engine once an entity is despawned and will change every time you restart the game.

If you need a **persistent** unique identifier (for saving/loading, multiplayer, or database syncing), you should define a custom component.

---

## 1. Using the Built-in `Entity` ID

If you only need a unique ID during a single runtime session, you can use the `Entity` type itself. It consists of an index and a generation (to prevent "aliasing" when IDs are recycled).

```rust
use bevy::prelude::*;

fn spawn_system(mut commands: Commands) {
    let entity = commands.spawn_empty().id();
    println!("Spawned entity with ID: {:?}", entity);
}

```

---

## 2. Creating a Persistent UUID Component

For IDs that stay the same across save files or network sessions, the standard approach is to create a `Component` wrapping a `u64` or a `UUID`.

### Using the `uuid` Crate

Add `uuid = { version = "1.0", features = ["v4"] }` to your `Cargo.toml`.

```rust
use bevy::prelude::*;
use uuid::Uuid;

#[derive(Component, Debug, Reflect)]
pub struct RuntimeId(pub Uuid);

impl Default for RuntimeId {
    fn default() -> Self {
        Self(Uuid::new_v4())
    }
}

fn setup(mut commands: Commands) {
    commands.spawn((
        RuntimeId::default(),
        Name::new("Player"),
    ));
}

```

---

## 3. Creating a Sequential ID Resource

If you want simple, human-readable incrementing IDs (1, 2, 3...), you can manage the "next ID" using a Bevy `Resource`.

```rust
#[derive(Resource, Default)]
struct IdCounter(u64);

#[derive(Component)]
struct SimpleId(u64);

fn spawn_with_id(mut commands: Commands, mut counter: ResMut<IdCounter>) {
    let new_id = counter.0;
    commands.spawn(SimpleId(new_id));
    counter.0 += 1;
}

```

---

## Which ID type should you use?

| ID Type | Use Case | Persistence | Performance |
| --- | --- | --- | --- |
| **`Entity`** | Internal logic, parent-child relations. | No | Fastest |
| **`u64` Counter** | Local save files, simple debugging. | Yes | Fast |
| **`Uuid` (v4)** | Multiplayer, merging save data, databases. | Yes | Moderate |

---

## Pro-Tip: Fast Lookups

By default, if you want to find an entity by your custom `RuntimeId`, you have to iterate through all entities in a query, which is .

If you need to do this frequently, I recommend using the **[bevy_mod_index](https://www.google.com/search?q=https://github.com/Andvane/bevy_mod_index)** crate. It allows you to create an index on your ID component for  lookups, similar to a primary key in a database.

Would you like me to show you how to implement a system to search for entities using these custom IDs?

To get  lookups with `bevy_mod_index`, you essentially treat your custom ID as a "database key." This avoids the performance hit of iterating through every entity () just to find one specific ID.

Here is how you implement it for a unique `u64` identifier.

---

## 1. Dependency Setup

Add the crate to your `Cargo.toml`:

```toml
[dependencies]
bevy = "0.14" # or latest
bevy_mod_index = "0.3"

```

## 2. Implementation Example

### Define the Component and the Index

The Index needs to know which component it is tracking and how to map it.

```rust
use bevy::prelude::*;
use bevy_mod_index::prelude::*;

#[derive(Component, Debug, Clone, Copy, PartialEq, Eq, Hash)]
struct MyId(u64);

// This type tells the library how to index 'MyId'
struct MyIdIndex;
impl IndexInfo for MyIdIndex {
    type Component = MyId;
    type Value = u64;
    type Storage = NoStorage<Self>;

    // How to extract the value we want to search by
    fn value(component: &Self::Component) -> Vec<Self::Value> {
        vec![component.0]
    }
}

```

### Register and Use the Index

You must register the index in your `App` and then use the `Index` system parameter to perform lookups.

```rust
fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        // 1. Register the index
        .add_index::<MyIdIndex>()
        .add_systems(Startup, spawn_entities)
        .add_systems(Update, lookup_system)
        .run();
}

fn spawn_entities(mut commands: Commands) {
    commands.spawn(MyId(101)).insert(Name::new("Hero"));
    commands.spawn(MyId(102)).insert(Name::new("Villain"));
}

fn lookup_system(index: Index<MyIdIndex>) {
    // 2. Perform the O(1) lookup
    let target_id = 101;
    let entities = index.lookup(&target_id);

    if let Some(entity) = entities.first() {
        println!("Found Entity {:?} with ID {}", entity, target_id);
    }
}

```

---

## Comparison: Query vs. Index

| Method | Syntax | Complexity | Performance |
| --- | --- | --- | --- |
| **Standard Query** | `query.iter().find( | c | c.0 == id)` |
| **bevy_mod_index** | `index.lookup(&id)` |  | Constant speed regardless of entity count. |

---

## Key Tips for `bevy_mod_index`:

* **Unique vs. Multi:** `index.lookup()` returns a `Vec<Entity>`. If your IDs are strictly unique, you can just use `.first()`.
* **Automatic Updates:** The index updates automatically whenever a `MyId` component is added, removed, or changed.
* **Storage Types:** In the example, I used `NoStorage`. If you have a very large number of entities, you can explore different storage backends provided by the crate for better memory management.

Would you like to see how to handle a "Many-to-One" relationship with this, such as finding all units belonging to a specific `PlayerID`?
