package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

public class Module {

    private final TalonFX driveMotor;
    private final TalonFX turnMotor;

    private final VelocityVoltage driveRequest = new VelocityVoltage(0);
    private final PositionVoltage turnRequest = new PositionVoltage(0);

    private static final double DRIVE_GEAR_RATIO = 5.9;
    private static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
    private static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
    public static final double FF = 2.54447;

    private final int corner;

    public Module(int corner) {
        this.corner = corner;

        switch (corner) {
            case 0 -> {
                driveMotor = new TalonFX(19);
                turnMotor = new TalonFX(18);
            }
            case 1 -> {
                driveMotor = new TalonFX(29);
                turnMotor = new TalonFX(28);
            }
            case 2 -> {
                driveMotor = new TalonFX(11);
                turnMotor = new TalonFX(12);
            }
            case 3 -> {
                driveMotor = new TalonFX(21);
                turnMotor = new TalonFX(22);
            }
            default -> throw new IllegalArgumentException("Invalid module index");
        }

        configureDriveMotor();
        configureTurnMotor();
    }

    private void configureDriveMotor() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        cfg.CurrentLimits.SupplyCurrentLimit = 40;
        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;

        cfg.Feedback.SensorToMechanismRatio = DRIVE_GEAR_RATIO;
        cfg.Feedback.RotorToSensorRatio = 1;

        cfg.Slot0.kP = 0.0;
        cfg.Slot0.kI = 0.0;
        cfg.Slot0.kD = 0.0;
        cfg.Slot0.kV = 0.0;

        driveMotor.getConfigurator().apply(cfg);
    }

    private void configureTurnMotor() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        cfg.CurrentLimits.SupplyCurrentLimit = 30;
        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;

        cfg.Feedback.SensorToMechanismRatio = 1.0;
        cfg.Feedback.RotorToSensorRatio = 1.0;

        cfg.Slot0.kP = 2.0;
        cfg.Slot0.kI = 0.0;
        cfg.Slot0.kD = 0.0;

        turnMotor.getConfigurator().apply(cfg);
    }

    public void setDesiredState(SwerveModuleState targetState, boolean shouldTurn) {
        SwerveModuleState optimized =
                SwerveModuleState.optimize(targetState, getAngle());

        double wheelRotPerSec = optimized.speedMetersPerSecond / WHEEL_CIRCUMFERENCE;

        driveMotor.setControl(driveRequest.withVelocity(wheelRotPerSec));

        if (shouldTurn) {
            turnMotor.setControl(turnRequest.withPosition(optimized.angle.getRadians()));
        }
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(turnMotor.getPosition().getValueAsDouble());
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
