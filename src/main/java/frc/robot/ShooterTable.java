package frc.robot;

import frc.robot.subsystems.Shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterTable {

    private final InterpolatingDoubleTreeMap angleMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap rpmMap = new InterpolatingDoubleTreeMap();

    public ShooterTable() {
        // Add tuned points here
        addPoint(2.0, 76, 3600);
        addPoint(3.0, 71, 3900);
        addPoint(4.0, 66, 4300);
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
