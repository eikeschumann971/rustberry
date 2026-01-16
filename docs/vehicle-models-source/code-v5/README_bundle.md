# Visibility Graph + Clothoid Lattice + Parking Maps (Bundle)

This bundle delivers:

1. **Reduced Visibility Graph** (robust predicates + spatial index)
2. **Clothoid‑like Lattice Primitive Generator** (short forward/reverse RS/clothoid ramps)
3. **Parking Lot Maps** (JSON) to benchmark **Hybrid A*** vs **Lattice**

## Build (single translation unit examples)

```bash
# Demo 1: visibility graph
c++ -O2 -std=c++17 \
  planning/vis/geometry.hpp planning/vis/spatial_hash.hpp \
  planning/vis/vis_graph.hpp planning/vis/vis_graph.cpp \
  vis_demo.cpp -o vis_demo

# Demo 2: primitives generator (writes YAML)
c++ -O2 -std=c++17 \
  planning/lattice/lattice_primitives.hpp planning/lattice/lattice_primitives.cpp \
  gen_primitives_demo.cpp -o gen_primitives_demo
```

## Run

```bash
# Visibility graph on a tight lot
./vis_demo maps/lot_tight.json out.graph.json

# Generate clothoid-like RS primitives
./gen_primitives_demo
# -> primitives/parking_clothoid_primitives.yaml
```

## Notes
- **Inflation**: obstacles in the JSON are *not* inflated for robot radius. Inflate upstream or grow polygons if needed.
- The visibility graph is **reduced** (start, goal, and **reflex vertices** only) and uses a **uniform-grid spatial hash** to accelerate visibility checks.
- The lattice generator outputs **short forward & reverse** primitives with **ramp‑hold‑ramp** curvature, approximating clothoids and compatible with nonholonomic A* over a lattice.
- Use the provided maps to compare **Hybrid A*** vs **Lattice**:
  - Hybrid A*: tune grid resolution (0.05–0.1 m), yaw bins (≥72), step length (0.25–0.5 m), RS heuristic.
  - Lattice: use the YAML primitives (or generate your own) and run A* over lattice states.

## Files
- `planning/vis/geometry.hpp` – robust predicates (orientation, intersection, point‑in‑polygon)
- `planning/vis/spatial_hash.hpp` – light uniform grid index for obstacle edges
- `planning/vis/vis_graph.hpp/.cpp` – reduced visibility graph builder
- `vis_demo.cpp` – example: read map JSON, write visibility graph JSON
- `planning/lattice/lattice_primitives.hpp/.cpp` – primitive generator & YAML writer
- `gen_primitives_demo.cpp` – example generator entrypoint
- `maps/lot_simple.json`, `maps/lot_tight.json`, `maps/lot_serpentine.json`
- `primitives/` – output folder for generated YAML

Enjoy! 🚗🅿️
