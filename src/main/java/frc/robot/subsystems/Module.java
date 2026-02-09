package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Module extends SubsystemBase {

    public final TalonFX driveMotor;
    public final TalonFX turnMotor;

    private final VelocityVoltage driveRequest = new VelocityVoltage(0).withSlot(0);
    private final PositionVoltage turnRequest = new PositionVoltage(0).withSlot(0);

    public static final double DRIVE_GEAR_RATIO = 6.75;
    public static final double STEER_GEAR_RATIO = 150.0 / 7.0;  // SDS MK4i exact ratio

    private static final double WHEEL_DIAMETER = Units.inchesToMeters(3.5);
    private static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;

    public final int corner;

    public Module(int corner) {
        this.corner = corner;

        switch (corner) {
            case 0 -> {
                driveMotor = new TalonFX(19, "5980");
                turnMotor = new TalonFX(18, "5980");
            }
            case 1 -> {
                driveMotor = new TalonFX(29, "5980");
                turnMotor = new TalonFX(28, "5980");
            }
            case 2 -> {
                driveMotor = new TalonFX(21, "5980");
                turnMotor = new TalonFX(22, "5980");
            }
            case 3 -> {
                driveMotor = new TalonFX(11, "5980");
                turnMotor = new TalonFX(12, "5980");
            }
            default -> throw new IllegalArgumentException("Invalid module index");
        }

        configureDriveMotor();
        configureTurnMotor();
    }

    private void configureDriveMotor() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        cfg.CurrentLimits.SupplyCurrentLimit = 50;
        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;

        cfg.Feedback.SensorToMechanismRatio = DRIVE_GEAR_RATIO;

        cfg.Slot0.kP = 0.12;
        cfg.Slot0.kI = 0.0;
        cfg.Slot0.kD = 0.0;
        cfg.Slot0.kV = 0.12;

        driveMotor.getConfigurator().apply(cfg);
    }

    private void configureTurnMotor() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // ⭐ CRITICAL FIX: Use the TalonFX integrated encoder, NOT the CANcoder
        cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

        // ⭐ Correct MK4i steering ratio
        cfg.Feedback.SensorToMechanismRatio = STEER_GEAR_RATIO;

        cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        cfg.Slot0.kP = 7.0;
        cfg.Slot0.kI = 0.0;
        cfg.Slot0.kD = 0.05;

        turnMotor.getConfigurator().apply(cfg);
    }

    // Manual zeroing (used only when wheels are physically straight)
    public void zeroSteerToForward() {
        turnMotor.setPosition(0.0);
    }

    // ⭐ Steering angle ALWAYS comes from the TalonFX integrated encoder
    public Rotation2d getAngle() {
        double mechRot = turnMotor.getPosition().getValueAsDouble();
        return Rotation2d.fromRotations(mechRot);
    }

    public void setDesiredState(SwerveModuleState targetState, boolean shouldTurn) {
        SwerveModuleState optimized =
                SwerveModuleState.optimize(targetState, getAngle());

        if (shouldTurn) {
            turnMotor.setControl(
                turnRequest.withPosition(optimized.angle.getRotations())
            );
        }

        double wheelRotPerSec =
                optimized.speedMetersPerSecond / WHEEL_CIRCUMFERENCE;

        driveMotor.setControl(
                driveRequest.withVelocity(wheelRotPerSec)
        );
    }

    public SwerveModuleState getState() {
        double wheelRotPerSec = driveMotor.getVelocity().getValueAsDouble();
        double speed = wheelRotPerSec * WHEEL_CIRCUMFERENCE;
        return new SwerveModuleState(speed, getAngle());
    }

    public SwerveModulePosition getPosition() {
        double wheelRot = driveMotor.getPosition().getValueAsDouble();
        double distance = wheelRot * WHEEL_CIRCUMFERENCE;
        return new SwerveModulePosition(distance, getAngle());
    }

    public void setIdleMode(NeutralModeValue mode) {
        driveMotor.setNeutralMode(mode);
        turnMotor.setNeutralMode(mode);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Module" + corner + " MotorAngleRot",
            turnMotor.getPosition().getValueAsDouble());

        SmartDashboard.putNumber("Module" + corner + " MotorAngleDeg",
            getAngle().getDegrees());
    }
}
