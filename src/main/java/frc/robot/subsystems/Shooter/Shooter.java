package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    private final TalonFX flywheel = new TalonFX(24, "5980");
    private final TalonFX hood = new TalonFX(23, "5980");

    private final CANcoder hoodEncoder = new CANcoder(8, "5980");

    private static final double FLYWHEEL_GEAR_RATIO = 1.0;
    private static final double HOOD_GEAR_RATIO = 2.0;

    public static double min_angle = 0.0;
    public static double max_angle;

    public static final double hoodOffsetLow = 0.3842; // <<<\replace with your measured absolute
    public static final double hoodOffsetHigh = 0.3842; // <<<\replace with your measured absolute

    private static final double RPM_TOL = 100;
    private static final double ANGLE_TOL = 1.0;

    private double goalRPM = 0;
    private double goalAngle = 0;

    private final VelocityVoltage flywheelReq = new VelocityVoltage(0).withEnableFOC(true);

    private final PositionVoltage hoodReq = new PositionVoltage(0).withEnableFOC(true);

    public Shooter() {
        max_angle = getAngleFromRotations(hoodOffsetHigh - hoodOffsetLow);

        System.out.println(max_angle);

        flywheel.setNeutralMode(NeutralModeValue.Coast);
        hood.setNeutralMode(NeutralModeValue.Brake);

        TalonFXConfiguration flyCfg = new TalonFXConfiguration();
        flyCfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        flyCfg.Slot0.kP = 0.18;
        flyCfg.Slot0.kI = 0.0;
        flyCfg.Slot0.kD = 0.0;
        flyCfg.Slot0.kV = 0.12;
        flyCfg.Slot0.kS = 0.0;
        flyCfg.Feedback.SensorToMechanismRatio = FLYWHEEL_GEAR_RATIO;
        flywheel.getConfigurator().apply(flyCfg, 0.25);

        TalonFXConfiguration hoodCfg = new TalonFXConfiguration();
        hoodCfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        hoodCfg.Slot0.kP = 0.5;
        hoodCfg.Slot0.kI = 0.0;
        hoodCfg.Slot0.kD = 0.0;
        hoodCfg.Feedback.SensorToMechanismRatio = HOOD_GEAR_RATIO;
        hood.getConfigurator().apply(hoodCfg, 0.25);

    }

    public double getAngleFromRotations(double rotations){
        return rotations * 360.0 / HOOD_GEAR_RATIO;
    }

    public double getRotationsFromAngle(double angle){
        return (angle / 360.0) * HOOD_GEAR_RATIO;
    }

    public void setFlywheelRPM(double rpm) {
        goalRPM = rpm;
        double motorRPS = -(rpm / 60.0) * FLYWHEEL_GEAR_RATIO;
        flywheel.setControl(flywheelReq.withVelocity(motorRPS));
    }

    public void setHoodAngle(double deg) {
        goalAngle = Math.max(min_angle, Math.min(max_angle, deg));
        hood.setControl(hoodReq.withPosition(getRotationsFromAngle(goalAngle)));
    }

    public double getFlywheelRPM() {
        double motorRPS = flywheel.getVelocity().getValueAsDouble();
        return (motorRPS / FLYWHEEL_GEAR_RATIO) * 60.0;
    }

    public double getHoodAngle() {
        double abs = hoodEncoder.getPosition().getValueAsDouble();
        double corrected = abs - hoodOffsetLow;

        return getAngleFromRotations(corrected);
    }

    public boolean readyToShoot() {
        return Math.abs(goalRPM - getFlywheelRPM()) < RPM_TOL &&
               Math.abs(goalAngle - getHoodAngle()) < ANGLE_TOL;
    }
}