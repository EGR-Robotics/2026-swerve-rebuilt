package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

public class Module {

    public final TalonFX driveMotor;
    public final TalonFX turnMotor;
    public final CANcoder angleEncoder;

    private final VelocityVoltage driveRequest = new VelocityVoltage(0).withSlot(0);
    private final PositionVoltage turnRequest = new PositionVoltage(0).withSlot(0);

    public static final double DRIVE_GEAR_RATIO = 6.75;
    public static final double STEER_GEAR_RATIO = 150.0 / 7.0;

    private static final double WHEEL_DIAMETER = Units.inchesToMeters(3.5);
    private static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;

    // Offsets ONLY come from ZeroWheels
    public static double[] STEER_OFFSETS = new double[4];

    public final int corner;

    public Module(int corner) {
        this.corner = corner;

        switch (corner) {
            case 0 -> {
                driveMotor = new TalonFX(19, "5980");
                turnMotor = new TalonFX(18, "5980");
                angleEncoder = new CANcoder(3, "5980");
            }
            case 1 -> {
                driveMotor = new TalonFX(29, "5980");
                turnMotor = new TalonFX(28, "5980");
                angleEncoder = new CANcoder(7, "5980");
            }
            case 2 -> {
                driveMotor = new TalonFX(21, "5980");
                turnMotor = new TalonFX(22, "5980");
                angleEncoder = new CANcoder(4, "5980");
            }
            case 3 -> {
                driveMotor = new TalonFX(11, "5980");
                turnMotor = new TalonFX(12, "5980");
                angleEncoder = new CANcoder(1, "5980");
            }
            default -> throw new IllegalArgumentException("Invalid module index");
        }

        configureDriveMotor();
        configureTurnMotor();
        configureCANcoder();
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
        cfg.Feedback.SensorToMechanismRatio = STEER_GEAR_RATIO;

        cfg.Slot0.kP = 8.0;
        cfg.Slot0.kI = 0.0;
        cfg.Slot0.kD = 0.18;

        cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        turnMotor.getConfigurator().apply(cfg);
    }

    private void configureCANcoder() {
        CANcoderConfiguration cfg = new CANcoderConfiguration();

        // ⭐ FIX: Make CANcoder direction match motor direction
        cfg.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        angleEncoder.getConfigurator().apply(cfg);
    }

    public void seedSteerFromAbsolute() {
        double absolute = angleEncoder.getAbsolutePosition().getValueAsDouble();
        double offset = STEER_OFFSETS[corner];
        double corrected = absolute - offset;
        double motorRot = corrected * STEER_GEAR_RATIO;
        turnMotor.setPosition(motorRot);
    }

    public Rotation2d getAngle() {
        double motorRot = turnMotor.getPosition().getValueAsDouble();
        double wheelRot = motorRot / STEER_GEAR_RATIO;
        return Rotation2d.fromRotations(wheelRot);
    }

    private double unwrap(double current, double target) {
        double diff = target - current;
        diff = Math.IEEEremainder(diff, 1.0);
        return current + diff;
    }

    public void setDesiredState(SwerveModuleState targetState, boolean shouldTurn) {
        double current = turnMotor.getPosition().getValueAsDouble() / STEER_GEAR_RATIO;

        double desiredWrapped = targetState.angle.getRotations();
        double desiredContinuous = unwrap(current, desiredWrapped);

        SwerveModuleState continuousState =
                new SwerveModuleState(targetState.speedMetersPerSecond,
                        Rotation2d.fromRotations(desiredContinuous));

        SwerveModuleState optimized =
                SwerveModuleState.optimize(continuousState, getAngle());

        if (shouldTurn) {
            turnMotor.setControl(turnRequest.withPosition(optimized.angle.getRotations()));
        }

        double wheelRotPerSec = optimized.speedMetersPerSecond / WHEEL_CIRCUMFERENCE;
        driveMotor.setControl(driveRequest.withVelocity(wheelRotPerSec));
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
}
