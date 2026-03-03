package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterTable {

    private final InterpolatingDoubleTreeMap angleMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap rpmMap = new InterpolatingDoubleTreeMap();

    public ShooterTable() {
    // Distances measured in feet on field, converted to meters for code
    // Format: addPoint(feet * 0.3048, angle, rpm) 
    // Use as many points as possible for more accuracy.
    addPoint(6  * 0.3048, 28.0, 3600);  // 6 feet
    addPoint(10 * 0.3048, 24.0, 3900);  // 10 feet
    addPoint(13 * 0.3048, 18.0, 4300);  // 13 feet
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