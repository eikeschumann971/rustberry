To solve this problem, you are looking for a **Voronoi Diagram**.

This is a classic computational geometry structure that partitions a plane into regions based on distance to a specific set of points (in your case, the road crossings).

### The Voronoi Partition

When you generate a Voronoi diagram for your crossings, every point within a specific polygon (called a **Voronoi cell**) is closer to its assigned crossing than to any other crossing in the network.

---

### Recommended Algorithms

Depending on your technical constraints and the size of your road network, you have three primary options:

| Algorithm | Complexity | Best For... |
| --- | --- | --- |
| **Fortune’s Algorithm** |  | **Efficiency.** It is a sweep-line algorithm and is the standard for large datasets. |
| **Lloyd’s Algorithm** | Iterative | **Uniformity.** If you want the crossings to be more "centered" in their polygons, this smooths the partitions. |
| **Delaunay Triangulation** |  | **Simplicity.** The Voronoi diagram is the "dual graph" of a Delaunay triangulation. Many libraries calculate this first and then flip it to get the Voronoi cells. |

### How to Implement It

If you aren't building the geometry engine from scratch, I recommend using existing libraries that handle the edge cases (like points being perfectly collinear):

1. **Python:** Use `scipy.spatial.Voronoi`. It is highly optimized and handles the underlying math using the QHULL library.
2. **JavaScript:** Use `d3-delaunay`. It is incredibly fast and designed specifically for partitioning planes in web applications.
3. **GIS Software:** If your road network is in a GIS format (like a Shapefile or GeoJSON), tools like **QGIS** or **ArcGIS** have "Create Voronoi Polygons" built-in as a standard geoprocessing tool.

### A Note on "The Boundary"

Voronoi diagrams naturally extend to infinity for the points on the "outside" of your cluster. To get clean polygons for every crossing, you will need to define a **bounding box** (the limits of your map) to clip those infinite lines into closed shapes.

---

Would you like me to provide a Python code snippet using `scipy` to demonstrate how to generate these polygons from a list of coordinates?


---

Yes, Rust has several high-quality crates for this. Because Voronoi diagrams are the "dual" of Delaunay triangulations, many of these libraries use a **Delaunay-to-Voronoi** approach, which is often faster and more robust than a raw Fortune's implementation in practice.

### Top Recommended Crates

#### 1. **voronoice** (Easiest to Use)

This is generally the best "all-in-one" choice for 2D plane partitioning. It is built on top of `delaunator` (a very fast triangulation engine) and handles the "infinite edge" problem by allowing you to define a bounding box easily.

* **Best for:** High performance and out-of-the-box polygon generation.
* **Key Feature:** Includes a simple `VoronoiBuilder` that supports **Lloyd's Relaxation** if you want to make your partitions more uniform.

#### 2. **spade** (Most Robust)

`spade` is a powerful computational geometry library that focuses on correctness and robustness. It uses precise geometric predicates to prevent crashes or errors caused by floating-point rounding (e.g., when three crossings are almost in a straight line).

* **Best for:** Production-grade geographic applications where precision is critical.
* **Key Feature:** Supports incremental updates—you can add or remove crossings without recalculating the entire diagram.

#### 3. **voronator**

A direct port of the popular `d3-delaunay` library from the JavaScript ecosystem.

* **Best for:** Developers coming from a web/D3 background.
* **Key Feature:** Very clean API for extracting the vertices of each polygon.

---

### Comparison at a Glance

| Crate | Algorithm Strategy | Primary Focus | Best Use Case |
| --- | --- | --- | --- |
| **voronoice** | Delaunay Dual | Speed & Convenience | General mapping / Game dev |
| **spade** | Delaunay Dual | Mathematical Robustness | GIS / Precision engineering |
| **voronator** | Delaunay Dual | D3 Compatibility | Data visualization |
| **boostvoronoi** | Fortune's (Sweep-line) | Feature parity with C++ | Segment-based Voronoi |

---

### Example with `voronoice`

If you choose `voronoice`, your implementation would look roughly like this:

```rust
use voronoice::*;

fn main() {
    // 1. Define your road crossing coordinates
    let sites = vec![
        Point { x: 10.0, y: 10.0 },
        Point { x: 20.0, y: 30.0 },
        Point { x: 50.0, y: 15.0 },
    ];

    // 2. Build the diagram with a bounding box (xmin, ymin, width, height)
    let my_voronoi = VoronoiBuilder::default()
        .set_sites(sites)
        .set_bounding_box(BoundingBox::new(Point { x: 0.0, y: 0.0 }, 100.0, 100.0))
        .build()
        .unwrap();

    // 3. Iterate through the resulting polygons
    for cell in my_voronoi.iter_cells() {
        let vertices: Vec<&Point> = cell.iter_vertices().collect();
        println!("Polygon for crossing at {:?} has vertices: {:?}", cell.site_position(), vertices);
    }
}

```

**Would you like me to show you how to handle "degenerate" cases—like when multiple crossings have the exact same coordinates—using one of these libraries?**

---

Yes, if your crossings form a **regular square grid**, the resulting Voronoi cells will be **perfect squares** with the crossing exactly at the center.

This happens because the Voronoi boundary is defined as the "perpendicular bisector" of the line segment connecting two neighboring points. In a square grid, these bisectors meet at right angles exactly halfway between the points, forming a new, identical grid shifted by half a unit.

---

### Important Nuances

While the math says they should be squares, there are three practical "gotchas" depending on your road network and implementation:

#### 1. The Boundary Effect

As mentioned before, Voronoi cells on the perimeter of your grid will technically extend to infinity. To make them look like squares, you must clip them with a **bounding box**.

* If your grid is  and your bounding box is tight to the points, the outer squares will be cut off.
* To see full squares for all points, your bounding box should extend at least half a grid-unit beyond the outermost points.

#### 2. Precision & "Degenerate" Cases

In a perfect square grid, there are points where **four** empty circles meet (the corners where four Voronoi cells touch).

* Algorithms like **Fortune's** or **Delaunay** sometimes struggle with this because they prefer "triangles" (three points meeting).
* Most modern Rust crates (like `spade` or `voronoice`) handle this automatically, but mathematically, a square grid is a "degenerate" case because the points are co-circular.

#### 3. Different Grid Types

If your road network follows different regular patterns, the shapes change:

* **Square Grid:** Results in **Squares**.
* **Hexagonal Grid:** Results in **Hexagons**.
* **Triangular Grid:** Results in **Hexagons** (this is how honeycomb structures are formed).

---

### Is your road network "Manhattan-style"?

If your roads are a perfect grid, the Voronoi partition will look exactly like a map of city blocks where the "crossing" is the center of the block.

However, if your "crossings" are actually the intersections of the roads, the Voronoi cells will represent the **influence area** of each intersection. In this case, the "streets" would actually be the edges of your Voronoi squares.

**Would you like to see how the shapes distort if you shift just one of the crossings slightly off-center?**

---

In the `voronoice` crate, the `BoundingBox::new` function does not accept a point cloud directly. It specifically requires a **center point**, a **width**, and a **height**.

However, calculating a bounding box from your point cloud is a common step. You can do this by finding the minimum and maximum  and  coordinates in your road network data.

### 1. Creating a Bounding Box from Points

If you have a list of points (your road crossings), you can find the extents and then construct the `BoundingBox`. Here is how you would do it in Rust:

```rust
use voronoice::{BoundingBox, Point};

fn get_bbox_from_points(points: &[Point]) -> BoundingBox {
    let min_x = points.iter().map(|p| p.x).fold(f64::INFINITY, f64::min);
    let max_x = points.iter().map(|p| p.x).fold(f64::NEG_INFINITY, f64::max);
    let min_y = points.iter().map(|p| p.y).fold(f64::INFINITY, f64::min);
    let max_y = points.iter().map(|p| p.y).fold(f64::NEG_INFINITY, f64::max);

    let width = max_x - min_x;
    let height = max_y - min_y;
    let center = Point {
        x: min_x + width / 2.0,
        y: min_y + height / 2.0,
    };

    // Note: To see the full square for outer points, 
    // you may want to add padding to width and height.
    BoundingBox::new(center, width, height)
}

```

### 2. Why `voronoice` uses this method

The library uses the bounding box primarily for **clipping**. Because a Voronoi diagram on an infinite plane has "open" cells on the edges (lines that go on forever), the `BoundingBox` provides a boundary to cut those lines and close the polygons.

### 3. Alternatives in other crates

If you find manually calculating the bounding box tedious, other crates handle this slightly differently:

* **`spade`**: Does not require a bounding box to compute the diagram. It handles the "infinite" edges using a special "infinite vertex." You only deal with clipping if you specifically need to render the shapes to a screen.
* **`voronator`**: Similar to `voronoice`, it often defaults to a unit square ( to ) unless you specify otherwise.

### Pro-Tip: Add Padding

If your road crossings are at the very edge of your data, their Voronoi cells will be cut exactly at the crossing point if you use a tight bounding box. To ensure every crossing is "inside" a full polygon, I recommend adding a small buffer:

```rust
let padding = 10.0;
BoundingBox::new(center, width + padding, height + padding)

```

Would you like me to help you write a full script that reads a list of coordinates (like from a CSV or JSON) and outputs the final polygon vertices?

---

To help you build a robust tool, I’ve put together a complete Rust example. This script uses `voronoice` and includes a helper function to automatically calculate the bounding box with padding so that your edge crossings aren't "cut off" at the points.

### The Rust Implementation

You'll need to add this to your `Cargo.toml`:

```toml
[dependencies]
voronoice = "0.2.0"

```

And here is the complete code:

```rust
use voronoice::*;

/// Calculates a BoundingBox that encompasses all points plus a margin.
fn calculate_auto_bbox(points: &[Point], margin: f64) -> BoundingBox {
    let mut min_x = f64::MAX;
    let mut max_x = f64::MIN;
    let mut min_y = f64::MAX;
    let mut max_y = f64::MIN;

    for p in points {
        if p.x < min_x { min_x = p.x; }
        if p.x > max_x { max_x = p.x; }
        if p.y < min_y { min_y = p.y; }
        if p.y > max_y { max_y = p.y; }
    }

    let width = (max_x - min_x) + (margin * 2.0);
    let height = (max_y - min_y) + (margin * 2.0);
    let center = Point {
        x: min_x + (max_x - min_x) / 2.0,
        y: min_y + (max_y - min_y) / 2.0,
    };

    BoundingBox::new(center, width, height)
}

fn main() {
    // 1. Your road crossings (example of a slightly noisy square grid)
    let crossings = vec![
        Point { x: 10.0, y: 10.0 }, Point { x: 20.0, y: 10.0 },
        Point { x: 10.0, y: 20.0 }, Point { x: 20.0, y: 20.5 }, // slight offset
    ];

    // 2. Automatically determine the box size
    // Padding ensures the outer crossings are fully enclosed in polygons
    let bbox = calculate_auto_bbox(&crossings, 5.0);

    // 3. Generate the Voronoi Diagram
    let v = VoronoiBuilder::default()
        .set_sites(crossings)
        .set_bounding_box(bbox)
        .build()
        .expect("Failed to build Voronoi diagram");

    // 4. Extract and print the polygons
    for (i, cell) in v.iter_cells().enumerate() {
        let site = cell.site_position();
        let vertices: Vec<(f64, f64)> = cell
            .iter_vertices()
            .map(|v| (v.x, v.y))
            .collect();

        println!("Crossing #{} at ({}, {})", i, site.x, site.y);
        println!("  Polygon Vertices: {:?}", vertices);
    }
}

```

---

### How the Geometry Works

The `iter_vertices()` method returns the points in order (clockwise or counter-clockwise), which allows you to draw them or save them as a polygon string for GIS tools.

### Handling "Degenerate" Grids

Since you mentioned a **regular square grid**, you should be aware that floating-point math can sometimes lead to "ghost" vertices if three or more points are perfectly equidistant.

* **The fix:** `voronoice` is generally stable, but if you notice tiny, near-zero-length edges in your squares, you may want to apply a small "snap-to-grid" or rounding to your output coordinates.

### Exporting the Data

If you need to move this data into a mapping tool like QGIS or a web map:

* **GeoJSON:** You would wrap those vertices into a `Polygon` feature.
* **WKT:** You can format the string as `POLYGON((x1 y1, x2 y2, ...))`.

Would you like me to add a snippet to export these results into a specific format like GeoJSON or CSV?

