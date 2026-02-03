package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

/**
 * SDS MK4i Swerve Module using:
 *  - Kraken X60 drive motor
 *  - Kraken X60 steer motor
 *  - CTRE CANcoder for absolute steering angle
 *  - CANivore bus
 *
 *  Compatible with your current WPILib version:
 *   - Uses optimize(desired, Rotation2d)
 *   - Avoids Phoenix 6 fields that vary by version
 *   - Fully commented for student learning
 */
public class Module {

    // -------------------------------
    // HARDWARE
    // -------------------------------
    private final TalonFX driveMotor;
    private final TalonFX turnMotor;
    private final CANcoder angleEncoder;

    // Phoenix 6 control requests
    private final VelocityVoltage driveRequest = new VelocityVoltage(0).withSlot(0);
    private final PositionVoltage turnRequest = new PositionVoltage(0).withSlot(0);

    // -------------------------------
    // SDS MK4i CONSTANTS
    // -------------------------------

    // MK4i L2 drive ratio (change if needed)
    private static final double DRIVE_GEAR_RATIO = 6.75;

    // MK4i steering ratio (same for all MK4i)
    private static final double STEER_GEAR_RATIO = 150.0 / 7.0; // ≈21.428

    // Wheel diameter (4")
    private static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
    private static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
    
    private static final double[] STEER_OFFSETS = {
            0.447754,
            0.001953,
            -0.306641,
            0.297119 
    };

    private final int corner;

    // -------------------------------
    // CONSTRUCTOR
    // -------------------------------
    public Module(int corner) {
        this.corner = corner;

        // -------------------------------
        // ASSIGN MOTOR + CANCODER IDs
        // AND PUT THEM ON THE CANIVORE BUS
        // -------------------------------
        switch (corner) {
            case 0 -> {
                driveMotor = new TalonFX(19, "5980");
                turnMotor = new TalonFX(18, "5980");
                angleEncoder = new CANcoder(3, "5980");
            }
            case 1 -> {
                driveMotor = new TalonFX(29, "5980");
                turnMotor = new TalonFX(28, "5980");
                angleEncoder = new CANcoder(0, "5980"); 
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

    // -------------------------------
    // CONFIG: DRIVE MOTOR
    // -------------------------------
    private void configureDriveMotor() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Current limiting
        cfg.CurrentLimits.SupplyCurrentLimit = 50;
        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;

        // Convert motor rotations → wheel meters
        cfg.Feedback.SensorToMechanismRatio = DRIVE_GEAR_RATIO;

        // PID + Feedforward tuning
        cfg.Slot0.kP = 0.12;
        cfg.Slot0.kI = 0.0;
        cfg.Slot0.kD = 0.0;
        cfg.Slot0.kV = 0.12;

        driveMotor.getConfigurator().apply(cfg);
    }

    // -------------------------------
    // CONFIG: TURN MOTOR
    // -------------------------------
    private void configureTurnMotor() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Steering gear ratio
        cfg.Feedback.SensorToMechanismRatio = STEER_GEAR_RATIO;

        // Steering PID tuning
        cfg.Slot0.kP = 6.0;
        cfg.Slot0.kI = 0.0;
        cfg.Slot0.kD = 0.1;

        turnMotor.getConfigurator().apply(cfg);
    }

    // -------------------------------
    // CONFIG: CANCODER (SAFE FOR ALL PHOENIX 6 VERSIONS)
    // -------------------------------
    private void configureCANcoder() {
        CANcoderConfiguration cfg = new CANcoderConfiguration();

        // We intentionally do NOT set AbsoluteSensorRange
        // because the field name changed across Phoenix 6 versions.
        // The default behavior works fine for swerve.

        angleEncoder.getConfigurator().apply(cfg);
    }

    // -------------------------------
    // GET ABSOLUTE ANGLE (radians)
    // -------------------------------
    private Rotation2d getAbsoluteAngle() {
        double angle = angleEncoder.getAbsolutePosition().getValueAsDouble();

        // Apply module-specific offset
        angle -= STEER_OFFSETS[corner];

        return Rotation2d.fromRotations(angle);
    }

    // -------------------------------
    // SET DESIRED STATE (YOUR WPILib VERSION)
    // -------------------------------
    public void setDesiredState(SwerveModuleState targetState, boolean shouldTurn) {

        // Your WPILib uses the old signature:
        //    optimize(desired, Rotation2d) --> will probably need to be changed if we update
        SwerveModuleState optimized =
                SwerveModuleState.optimize(targetState, getAngle());

        // Convert m/s → wheel rotations per second
        double wheelRotPerSec = optimized.speedMetersPerSecond / WHEEL_CIRCUMFERENCE;

        // Drive velocity control
        driveMotor.setControl(driveRequest.withVelocity(wheelRotPerSec));

        // Steering position control
        if (shouldTurn) {
            turnMotor.setControl(turnRequest.withPosition(optimized.angle.getRotations()));
        }
    }

    // -------------------------------
    // GET CURRENT STEERING ANGLE
    // -------------------------------
    public Rotation2d getAngle() {
        return getAbsoluteAngle();
    }

    // -------------------------------
    // GET MODULE STATE
    // -------------------------------
    public SwerveModuleState getState() {
        double wheelRotPerSec = driveMotor.getVelocity().getValueAsDouble();
        double speed = wheelRotPerSec * WHEEL_CIRCUMFERENCE;

        return new SwerveModuleState(speed, getAngle());
    }

    // -------------------------------
    // GET MODULE POSITION (for odometry)
    // -------------------------------
    public SwerveModulePosition getPosition() {
        double wheelRot = driveMotor.getPosition().getValueAsDouble();
        double distance = wheelRot * WHEEL_CIRCUMFERENCE;

        return new SwerveModulePosition(distance, getAngle());
    }

    // -------------------------------
    // SET BRAKE/COAST MODE
    // -------------------------------
    public void setIdleMode(NeutralModeValue mode) {
        driveMotor.setNeutralMode(mode);
        turnMotor.setNeutralMode(mode);
    }
}