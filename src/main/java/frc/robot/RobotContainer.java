package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AutoFaceHubCommand;
import frc.robot.commands.FeedFromNeutral;
import frc.robot.commands.FeedFromOpposite;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); 
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); 
    private double MAX_RPM = 5000;

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public Shooter shooter = new Shooter();
    public Climber climber = new Climber();
    public Intake intake = new Intake();
    public Vision vision = new Vision();


    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {

        drivetrain.setVision(vision);

        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driverController.getLeftY() * MaxSpeed)
                    .withVelocityY(-driverController.getLeftX() * MaxSpeed)
                    .withRotationalRate(-driverController.getRightX() * MaxAngularRate)
            )
        );

        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // ---------------- DRIVER CONTROLS ----------------

        driverController.leftTrigger().whileTrue(
            new RunCommand(() -> intake.intake(driverController.getLeftTriggerAxis()), intake)
                .finallyDo(() -> intake.stopRoller())
        );

        driverController.leftBumper().whileTrue(
            new RunCommand(() -> intake.reverseIntake(), intake)
                .finallyDo(() -> intake.stopRoller())
        );

        driverController.rightTrigger().whileTrue(
            new RunCommand(() -> {

                boolean autoAlignActive =
                    driverController.x().getAsBoolean() ||
                    driverController.b().getAsBoolean();

                if (autoAlignActive) {
                    shooter.setFeederSpeed(3000);
                    shooter.setRollerRPM(3000);
                } else {
                    shooter.setFeederSpeed(3000);
                    shooter.setRollerRPM(3000);
                    shooter.setFlywheelRPM(800); // tune spit-out RPM 
                }

            }, shooter).finallyDo(() -> {
                shooter.stopFeeder();
                shooter.stopRoller();
                shooter.setFlywheelRPM(0);
            })
        );

        //Consider changigng so we can make y a preset shoot in hub angle + speed
        driverController.y().onTrue(
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric())
        );

        driverController.x().whileTrue(new FeedFromNeutral(shooter));
        driverController.b().whileTrue(new FeedFromOpposite(shooter));
        
        //Eventually will add autoshootaligncommand to this, but for testing first leave just autofacehub
        driverController.a().whileTrue(
            new AutoFaceHubCommand(drivetrain)
        );


        driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // ---------------- OPERATOR CONTROLS ----------------

        shooter.setDefaultCommand(
            new RunCommand(() -> {
                double hoodTrigger = operatorController.getLeftTriggerAxis();
                if (hoodTrigger > 0.05) {
                    double minAngle = 40;
                    double maxAngle = 80;
                    double angle = maxAngle - hoodTrigger * (maxAngle - minAngle);
                    shooter.setHoodAngle(angle);
                }
            }, shooter)
        );

        operatorController.rightTrigger().whileTrue(
            new RunCommand(() -> {
                double flywheelTrigger = operatorController.getRightTriggerAxis();
                double rpm = flywheelTrigger * MAX_RPM;
                shooter.feedAndFlywheel(rpm);
            }, shooter).finallyDo(() -> shooter.feedAndFlywheel(0))
        );

        operatorController.y().onTrue(
            new InstantCommand(() -> climber.toggleDirection(), climber)
        );

        operatorController.b().whileTrue(
            new RunCommand(() -> climber.climbRight(), climber)
                .finallyDo(() -> climber.stop())
        );

        operatorController.a().whileTrue(
            new RunCommand(() -> climber.climbBoth(), climber)
                .finallyDo(() -> climber.stop())
        );

        operatorController.x().whileTrue(
            new RunCommand(() -> climber.climbLeft(), climber)
                .finallyDo(() -> climber.stop())
        );

        operatorController.leftBumper().whileTrue(
            new RunCommand(() -> intake.lowerIntake(), intake)
                .finallyDo(() -> intake.stopPivot())
        );

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0.5)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            )
            .withTimeout(5.0),
            drivetrain.applyRequest(() -> idle)
        );
    }
}
