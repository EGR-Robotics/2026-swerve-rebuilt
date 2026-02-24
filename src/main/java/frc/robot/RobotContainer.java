package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Shooter;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private double MAX_RPM = 5000;

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public Shooter shooter = new Shooter();
    public Climber climber = new Climber();
    public Intake intake = new Intake();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverController.getLeftX() * MaxSpeed) // Drive left  with negative X (left)
                    .withRotationalRate(-driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driverController.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        //joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        driverController.leftBumper().onTrue(
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric())   // *** CHANGED ***
        );

        // Set field orientation using the Y button
        driverController.y().onTrue(
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric())
        );

        // Calibrate module offsets on Right Bumper
        driverController.rightBumper().onTrue(
            new InstantCommand(() -> drivetrain.calibrateOffsets())   // *** ADDED ***
        );

        shooter.setDefaultCommand(
            new RunCommand(() -> {
                double flywheelTrigger = operatorController.getRightTriggerAxis();
                double rpm = flywheelTrigger * MAX_RPM; // TODO: change max rpm number
                shooter.setFlywheelRPM(rpm);
                // shooter.setFeederSpeed(flywheelTrigger);

                double hoodTrigger = operatorController.getLeftTriggerAxis();
                double minAngle = 40;// TODO: change the values of angles
                double maxAngle = 80;

                double angle = maxAngle - hoodTrigger * (maxAngle - minAngle);
                shooter.setHoodAngle(angle);
            }, shooter)
        );

        // driverController.x().whileTrue(new RunCommand(() -> shooter.feedAndFlywheel(5000), shooter));

        operatorController.b().whileTrue(new RunCommand(() -> climber.climbRight(), climber).finallyDo(() -> climber.stop()));
        operatorController.x().whileTrue(new RunCommand(() -> climber.climbLeft(), climber).finallyDo(() -> climber.stop()));
        operatorController.a().whileTrue(new RunCommand(() -> climber.climbBoth(), climber).finallyDo(() -> climber.stop()));
        operatorController.y().whileTrue(new RunCommand(() -> climber.toggleDirection(), climber));
        
        driverController.leftTrigger().whileTrue(new RunCommand(() -> intake.intake(driverController.getLeftTriggerAxis()), intake).finallyDo(() -> intake.stopRoller()));
        driverController.leftBumper().whileTrue(new RunCommand(() -> intake.reverseIntake(), intake).finallyDo(() -> intake.stopRoller()));
        
        operatorController.leftBumper().whileTrue(new RunCommand(() -> intake.lowerIntake(), intake).finallyDo(() -> intake.stopPivot()));//for testing
        
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
            // Reset our field centric heading to match the robot
            // facing away from our alliance station wall (0 deg).
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            // Then slowly drive forward (away from us) for 5 seconds.
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0.5)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            )
            .withTimeout(5.0),
            // Finally idle for the rest of auton
            drivetrain.applyRequest(() -> idle)
        );
    }
}
