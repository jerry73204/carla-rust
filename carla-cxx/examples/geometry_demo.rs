use carla_cxx::{
    SimpleBoundingBox, SimpleLocation, SimpleRotation, SimpleTransform, SimpleVector2D,
    SimpleVector3D,
};

fn main() -> anyhow::Result<()> {
    println!("=== CARLA-CXX Geometry Types Demo ===\n");

    // Vector2D demonstrations
    println!("📐 Vector2D Operations:");
    let v2d_a = SimpleVector2D::new(3.0, 4.0);
    let v2d_b = SimpleVector2D::new(1.0, 2.0);

    println!("  Vector A: ({}, {})", v2d_a.x, v2d_a.y);
    println!("  Vector B: ({}, {})", v2d_b.x, v2d_b.y);
    println!("  A + B = {:?}", v2d_a + v2d_b);
    println!("  A - B = {:?}", v2d_a - v2d_b);
    println!("  A * 2.0 = {:?}", v2d_a * 2.0);
    println!("  Length of A = {:.2}", v2d_a.length());
    println!("  Dot product A·B = {:.2}", v2d_a.dot(&v2d_b));
    println!("  Distance A to B = {:.2}", v2d_a.distance_to(&v2d_b));
    println!("  Normalized A = {:?}\n", v2d_a.normalized());

    // Vector3D demonstrations
    println!("📐 Vector3D Operations:");
    let v3d_a = SimpleVector3D::new(1.0, 2.0, 3.0);
    let v3d_b = SimpleVector3D::new(4.0, 5.0, 6.0);

    println!("  Vector A: ({}, {}, {})", v3d_a.x, v3d_a.y, v3d_a.z);
    println!("  Vector B: ({}, {}, {})", v3d_b.x, v3d_b.y, v3d_b.z);
    println!("  A + B = {:?}", v3d_a + v3d_b);
    println!("  A - B = {:?}", v3d_a - v3d_b);
    println!("  A * 2.0 = {:?}", v3d_a * 2.0);
    println!("  Length of A = {:.2}", v3d_a.length());
    println!("  Dot product A·B = {:.2}", v3d_a.dot(&v3d_b));
    println!("  Cross product A×B = {:?}", v3d_a.cross(&v3d_b));
    println!("  Distance A to B = {:.2}", v3d_a.distance_to(&v3d_b));
    println!("  Normalized A = {:?}", v3d_a.normalized());
    println!("  A as 2D = {:?}\n", v3d_a.to_2d());

    // Location demonstrations
    println!("📍 Location Operations:");
    let loc_a = SimpleLocation::new(10.0, 20.0, 5.0);
    let loc_b = SimpleLocation::new(15.0, 25.0, 8.0);
    let offset = SimpleVector3D::new(2.0, 3.0, 1.0);

    println!("  Location A: ({}, {}, {})", loc_a.x, loc_a.y, loc_a.z);
    println!("  Location B: ({}, {}, {})", loc_b.x, loc_b.y, loc_b.z);
    println!("  Offset: ({}, {}, {})", offset.x, offset.y, offset.z);
    println!("  A + offset = {:?}", loc_a + offset);
    println!("  A - offset = {:?}", loc_a - offset);
    println!("  B - A = {:?}", loc_b - loc_a);
    println!("  Distance A to B = {:.2}", loc_a.distance_to(&loc_b));
    println!("  Translate A by offset = {:?}\n", loc_a.translate(&offset));

    // Rotation demonstrations
    println!("🔄 Rotation Operations:");
    let rot_degrees = SimpleRotation::from_degrees(45.0, 90.0, 180.0);
    let rot_radians = SimpleRotation::from_radians(
        std::f64::consts::PI / 4.0,
        std::f64::consts::PI / 2.0,
        std::f64::consts::PI,
    );

    println!(
        "  Rotation from degrees (45°, 90°, 180°): {:?}",
        rot_degrees
    );
    println!("  Rotation from radians (π/4, π/2, π): {:?}", rot_radians);

    let (pitch_rad, yaw_rad, roll_rad) = rot_degrees.to_radians();
    println!(
        "  Degrees to radians: ({:.4}, {:.4}, {:.4})",
        pitch_rad, yaw_rad, roll_rad
    );

    // Transform demonstrations
    println!("\n🔧 Transform Operations:");
    let transform = SimpleTransform::new(loc_a, rot_degrees);

    println!(
        "  Transform: location={:?}, rotation={:?}",
        transform.location, transform.rotation
    );

    // Transform operations would require CARLA server connection for the FFI calls
    // We can demonstrate the local operations
    let translated_transform = transform.translate(&offset);
    println!(
        "  Translated by offset: location={:?}",
        translated_transform.location
    );

    let additional_rotation = SimpleRotation::new(10.0, 20.0, 30.0);
    let rotated_transform = transform.rotate(&additional_rotation);
    println!(
        "  Rotated additionally: rotation={:?}",
        rotated_transform.rotation
    );

    // BoundingBox demonstrations
    println!("\n📦 BoundingBox Operations:");
    let bbox_center = SimpleLocation::new(0.0, 0.0, 0.0);
    let bbox_extent = SimpleVector3D::new(5.0, 10.0, 2.0);
    let bbox = SimpleBoundingBox::new(bbox_center, bbox_extent);

    println!("  BoundingBox center: {:?}", bbox.get_center());
    println!("  BoundingBox extent: {:?}", bbox.get_extent());
    println!("  BoundingBox min: {:?}", bbox.get_min());
    println!("  BoundingBox max: {:?}", bbox.get_max());

    // Test point containment
    let test_points = [
        SimpleLocation::new(0.0, 0.0, 0.0),    // center - should be inside
        SimpleLocation::new(4.0, 9.0, 1.0),    // inside
        SimpleLocation::new(6.0, 11.0, 3.0),   // outside
        SimpleLocation::new(-3.0, -8.0, -1.0), // inside
    ];

    for (i, point) in test_points.iter().enumerate() {
        println!(
            "  Point {}: {:?} - {}",
            i,
            point,
            if bbox.contains(point) {
                "INSIDE"
            } else {
                "OUTSIDE"
            }
        );
    }

    let expanded_bbox = bbox.expand(2.0);
    println!(
        "  Expanded BoundingBox extent: {:?}",
        expanded_bbox.get_extent()
    );

    // Constants demonstration
    println!("\n🎯 Geometry Constants:");
    println!("  Vector2D::ZERO = {:?}", SimpleVector2D::ZERO);
    println!("  Vector2D::UNIT_X = {:?}", SimpleVector2D::UNIT_X);
    println!("  Vector2D::UNIT_Y = {:?}", SimpleVector2D::UNIT_Y);
    println!("  Vector3D::ZERO = {:?}", SimpleVector3D::ZERO);
    println!("  Vector3D::UNIT_X = {:?}", SimpleVector3D::UNIT_X);
    println!("  Vector3D::UNIT_Y = {:?}", SimpleVector3D::UNIT_Y);
    println!("  Vector3D::UNIT_Z = {:?}", SimpleVector3D::UNIT_Z);
    println!("  Location::ZERO = {:?}", SimpleLocation::ZERO);
    println!("  Rotation::ZERO = {:?}", SimpleRotation::ZERO);
    println!("  Transform::IDENTITY = {:?}", SimpleTransform::IDENTITY);

    // Conversion demonstrations
    println!("\n🔄 Type Conversions:");
    let tuple_2d: SimpleVector2D = (7.0, 8.0).into();
    let tuple_3d: SimpleVector3D = (9.0, 10.0, 11.0).into();
    let tuple_loc: SimpleLocation = (12.0, 13.0, 14.0).into();
    let tuple_rot: SimpleRotation = (15.0, 16.0, 17.0).into();

    println!("  From tuple (7, 8) to Vector2D: {:?}", tuple_2d);
    println!("  From tuple (9, 10, 11) to Vector3D: {:?}", tuple_3d);
    println!("  From tuple (12, 13, 14) to Location: {:?}", tuple_loc);
    println!("  From tuple (15, 16, 17) to Rotation: {:?}", tuple_rot);

    let vector_to_location: SimpleLocation = tuple_3d.into();
    let location_to_vector: SimpleVector3D = tuple_loc.into();
    println!("  Vector3D to Location: {:?}", vector_to_location);
    println!("  Location to Vector3D: {:?}", location_to_vector);

    println!("\n✅ Geometry demo completed successfully!");
    println!("💡 Note: Some advanced transform operations (like get_forward_vector) require");
    println!("   a connection to CARLA server for the FFI calls to work.");

    Ok(())
}
