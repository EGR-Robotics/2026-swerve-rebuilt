package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    private final TalonFX flywheel = new TalonFX(30); // Kraken 60X
    private final TalonFX hood = new TalonFX(31);     // Kraken X44

    private static final double FLYWHEEL_GEAR_RATIO = 1.0;
    private static final double HOOD_GEAR_RATIO = 50.0;

    private static final double MIN_ANGLE = 40;
    private static final double MAX_ANGLE = 80;

    private static final double RPM_TOL = 100;
    private static final double ANGLE_TOL = 1.0;

    private double goalRPM = 0;
    private double goalAngle = 60;

    private final VelocityVoltage flywheelReq = new VelocityVoltage(0).withEnableFOC(true);
    private final PositionVoltage hoodReq = new PositionVoltage(0).withEnableFOC(true);

    public Shooter() {
        flywheel.setNeutralMode(NeutralModeValue.Coast);
        hood.setNeutralMode(NeutralModeValue.Brake);
    }

    // -------------------------
    // Public API
    // -------------------------

    public void setFlywheelRPM(double rpm) {
        goalRPM = rpm;
        double motorRPS = (rpm / 60.0) * FLYWHEEL_GEAR_RATIO;
        flywheel.setControl(flywheelReq.withVelocity(motorRPS));
    }

    public void setHoodAngle(double deg) {
        deg = Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, deg));
        goalAngle = deg;
        double motorRot = (deg / 360.0) * HOOD_GEAR_RATIO;
        hood.setControl(hoodReq.withPosition(motorRot));
    }

    public double getFlywheelRPM() {
        double motorRPS = flywheel.getVelocity().getValueAsDouble();
        return (motorRPS / FLYWHEEL_GEAR_RATIO) * 60.0;
    }

    public double getHoodAngle() {
        double motorRot = hood.getPosition().getValueAsDouble();
        return (motorRot / HOOD_GEAR_RATIO) * 360.0;
    }

    public boolean readyToShoot() {
        return Math.abs(goalRPM - getFlywheelRPM()) < RPM_TOL &&
               Math.abs(goalAngle - getHoodAngle()) < ANGLE_TOL;
    }
}
