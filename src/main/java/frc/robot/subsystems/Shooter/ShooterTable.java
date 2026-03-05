package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterTable {

    private final InterpolatingDoubleTreeMap angleMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap rpmMap = new InterpolatingDoubleTreeMap();

    public ShooterTable() {
    // Distances measured in feet on field, converted to meters for code
    // Format: addPoint(feet * 0.3048, angle, rpm) 
    // Use as many points as possible for more accuracy.
    addPoint(3  * 0.3048, 0.0, 2150);  // 3 feet
    addPoint(3.5  * 0.3048, 0.0, 2150);  // 3.5 feet
    addPoint(4  * 0.3048, 0.0, 2175);  // 4 feet
    addPoint(4.5  * 0.3048, 0.0, 2200);  // 4.5 feet
    addPoint(5  * 0.3048, 0.0, 2215);  // 5 feet
    addPoint(5.5  * 0.3048, 0.0, 2225);  // 5.5 feet
    addPoint(6  * 0.3048, 0.0, 2225);  // 6 feet
    addPoint(6.5  * 0.3048, 0.0, 2235);  // 6.5 feet
}

    private void addPoint(double distance, double angle, double rpm) {
        angleMap.put(distance, angle);
        rpmMap.put(distance, rpm);
    }

    public ShooterSetpoint getSetpoint(double distance) {
        return new ShooterSetpoint(
            angleMap.get(distance),
            rpmMap.get(distance)
        );
    }

    public static record ShooterSetpoint(double angle, double rpm) {}
}