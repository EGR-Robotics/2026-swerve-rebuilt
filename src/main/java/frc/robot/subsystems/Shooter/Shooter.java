package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    private final TalonFX flywheel = new TalonFX(24, "5980");
    private final TalonFX hood = new TalonFX(23, "5980");

    private final CANcoder hoodEncoder = new CANcoder(8, "5980");

    private static final double FLYWHEEL_GEAR_RATIO = 1.0;

    // REAL encoder readings after switching to Unsigned_0To1 in Tuner X
    // IMPORTANT: encoderMin = LOWER value (hood UP)
    //            encoderMax = HIGHER value (hood DOWN)
    private static final double encoderMin = 0.25;   // hood UP
    private static final double encoderMax = 0.5;   // hood DOWN

    public static final double min_angle = 0.0;
    public static double max_angle = 30.0;

    // Hood motor rotates 1.8 turns from 0° → 240°
    private static final double MOTOR_ROTATIONS_AT_MAX = 2.05;

    private static final double RPM_TOL = 100;
    private static final double ANGLE_TOL = 1.0;

    private double goalRPM = 0;
    private double goalAngle = 0;

    private final VelocityVoltage flywheelReq = new VelocityVoltage(0).withEnableFOC(true);
    private final PositionVoltage hoodReq = new PositionVoltage(0).withEnableFOC(true);

    public Shooter() {

        flywheel.setNeutralMode(NeutralModeValue.Coast);
        hood.setNeutralMode(NeutralModeValue.Brake);

        // Flywheel config
        TalonFXConfiguration flyCfg = new TalonFXConfiguration();
        flyCfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        flyCfg.Slot0.kP = 0.01;
        flyCfg.Slot0.kI = 0.0;
        flyCfg.Slot0.kD = 0.0;
        flyCfg.Slot0.kV = 0.12;
        flyCfg.Slot0.kS = 0.0;
        flyCfg.Feedback.SensorToMechanismRatio = FLYWHEEL_GEAR_RATIO;
        flywheel.getConfigurator().apply(flyCfg, 0.25);

        // Hood config
        TalonFXConfiguration hoodCfg = new TalonFXConfiguration();

        hoodCfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        hoodCfg.Feedback.FeedbackRemoteSensorID = hoodEncoder.getDeviceID();

        // Disable motor encoder contribution
        hoodCfg.Feedback.RotorToSensorRatio = 0.0;

        // CANcoder rotations = mechanism rotations
        hoodCfg.Feedback.SensorToMechanismRatio = 1.0;

        // No blending
        hoodCfg.Feedback.FeedbackRotorOffset = 0.0;

        // PID
        hoodCfg.Slot0.kP = 2.0;
        hoodCfg.Slot0.kI = 0.0;
        hoodCfg.Slot0.kD = 0.0;

        // Motor direction
        hoodCfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        hood.getConfigurator().apply(hoodCfg, 0.25);

        // CANcoder config (range already set in Tuner X)
        CANcoderConfiguration encCfg = new CANcoderConfiguration();
        hoodEncoder.getConfigurator().apply(encCfg);
    }

    // Convert hood angle → motor rotations (1.8 rotations = 240 degrees)
    private double angleToMotorRot(double angleDeg) {
        double pct = angleDeg / max_angle;
        return pct * MOTOR_ROTATIONS_AT_MAX;
    }

    // Convert CANcoder absolute → hood angle
    private double encoderToAngle(double enc) {
        double pct = (enc - encoderMin) / (encoderMax - encoderMin);
        return pct * max_angle;
    }

    public void setFlywheelRPM(double rpm) {
        goalRPM = rpm;
        double motorRPS = -(rpm / 60.0) * FLYWHEEL_GEAR_RATIO;
        flywheel.setControl(flywheelReq.withVelocity(motorRPS));
    }

    public void setHoodAngle(double deg) {
        System.out.println(getHoodAngle());
        
        goalAngle = Math.max(min_angle, Math.min(max_angle, deg));

        double motorRot = angleToMotorRot(goalAngle);

        hood.setControl(hoodReq.withPosition(motorRot));
    }

    public double getFlywheelRPM() {
        double motorRPS = flywheel.getVelocity().getValueAsDouble();
        return (motorRPS / FLYWHEEL_GEAR_RATIO) * 60.0;
    }

    public double getHoodAngle() {
        double abs = hoodEncoder.getAbsolutePosition().getValueAsDouble();
        return encoderToAngle(abs);
    }

    // public void stopFlywheelandHood() {
    //     setHoodAngle(10);
    // }

    public boolean readyToShoot() {
        return Math.abs(goalRPM - getFlywheelRPM()) < RPM_TOL &&
               Math.abs(goalAngle - getHoodAngle()) < ANGLE_TOL;
    }
}
