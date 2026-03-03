package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.ByOutpostShot;
import frc.robot.commands.AutoFaceHubCommand;
import frc.robot.commands.FeedFromNeutral;
import frc.robot.commands.FeedFromOpposite;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber; 
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Feed;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Shooter.Shooter;

import java.util.List;

public class RobotContainer {

    private double MaxSpeed = 0.6 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
    private double MAX_RPM = 5000;

    private final SlewRateLimiter xLimiter = new SlewRateLimiter(1.5);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(1.5);
    private final SlewRateLimiter rotLimiter = new SlewRateLimiter(1.5);

    private double shape(double input) {
        return Math.copySign(input * input * input, input);
    }

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
    public Feed feed = new Feed();
    public Intake intake = new Intake();
    public Vision vision = new Vision();

    // ---------------- AUTO SELECTORS ----------------
    private SendableChooser<String> allianceChooser = new SendableChooser<>();
    private SendableChooser<String> autoChooser = new SendableChooser<>();

    // Add new autos here to make them appear in the dropdown
    private final List<String> redAutos = List.of("Red 1", "Red 2", "Red 3");
    private final List<String> blueAutos = List.of("Blue 1", "Blue 2", "Blue 3");

    public RobotContainer() {
        configureBindings();
        configureAutoSelectors(); // sets up SmartDashboard auto UI
    }

    private void configureAutoSelectors() {

        // Alliance selector (multiple choice)
        allianceChooser.setDefaultOption("Red Alliance", "RED");
        allianceChooser.addOption("Blue Alliance", "BLUE");
        SmartDashboard.putData("Alliance Selector", allianceChooser);

        // Build the initial auto list (default = red)
        populateAutoChooser("RED");
        SmartDashboard.putData("Auto Selector", autoChooser);

        // When alliance changes, rebuild the auto dropdown
        allianceChooser.onChange(alliance -> populateAutoChooser(alliance));
    }

    private void populateAutoChooser(String alliance) {
        // WPILib requires creating a new chooser to update options
        SendableChooser<String> newChooser = new SendableChooser<>();

        if (alliance.equals("RED")) {
            for (String auto : redAutos) newChooser.addOption(auto, auto);
            newChooser.setDefaultOption(redAutos.get(0), redAutos.get(0));
        } else {
            for (String auto : blueAutos) newChooser.addOption(auto, auto);
            newChooser.setDefaultOption(blueAutos.get(0), blueAutos.get(0));
        }

        autoChooser = newChooser;
        SmartDashboard.putData("Auto Selector", autoChooser);
    }

    private void configureBindings() {

        drivetrain.setVision(vision);

        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {

                double x = -driverController.getLeftY();
                double y = -driverController.getLeftX();
                double rot = -driverController.getRightX();

                double shapedX = shape(x);
                double shapedY = shape(y);
                double shapedRot = shape(rot);

                double limitedX = xLimiter.calculate(shapedX);
                double limitedY = yLimiter.calculate(shapedY);
                double limitedRot = rotLimiter.calculate(shapedRot);

                return drive.withVelocityX(limitedX * MaxSpeed)
                            .withVelocityY(limitedY * MaxSpeed)
                            .withRotationalRate(limitedRot * MaxAngularRate);
            })
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

        driverController.rightBumper().whileTrue(
            new RunCommand(() -> feed.reverseRoller(9000), feed)
                .finallyDo(() -> feed.stopRoller())
        );

        driverController.rightTrigger().whileTrue(
            new RunCommand(() -> feed.feedFuel(9000), 
            feed).finallyDo(() -> feed.stopFeed(0))
        );

        driverController.povUp().onTrue(
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric())
        );

        driverController.y().whileTrue(new ByOutpostShot(shooter));
        driverController.b().whileTrue(new FeedFromNeutral(shooter));
        driverController.x().whileTrue(new FeedFromOpposite(shooter));
        
        driverController.a().whileTrue(new AutoFaceHubCommand(drivetrain, vision));

        driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // ---------------- OPERATOR CONTROLS ----------------
        
        operatorController.leftTrigger().whileTrue(
            new RunCommand(() -> {
                double trigger = operatorController.getLeftTriggerAxis();
                double angle = trigger * Shooter.max_angle;
                shooter.setHoodAngle(angle);
            }, shooter)
        ).onFalse(
            new InstantCommand(() -> shooter.setHoodAngle(Shooter.min_angle), shooter)
        );

        operatorController.rightTrigger().whileTrue(
            new RunCommand(() -> {
                double flywheelTrigger = operatorController.getRightTriggerAxis();
                double rpm = flywheelTrigger * MAX_RPM;
                shooter.setFlywheelRPM(rpm);
            }, shooter).finallyDo(() -> shooter.setFlywheelRPM(0))
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
        ).onFalse(
            new InstantCommand(() -> intake.stopPivot(), intake)
        );

        operatorController.rightBumper().whileTrue(
            new RunCommand(() -> intake.raiseIntake(), intake)
        ).onFalse(
            new InstantCommand(() -> intake.stopPivot(), intake)
        );


        drivetrain.registerTelemetry(logger::telemeterize);
    }

    // ---------------- AUTO COMMAND SELECTION ----------------
    public Command getAutonomousCommand() {
        // Whatever is selected on SmartDashboard WILL run in auto
        String alliance = allianceChooser.getSelected();
        String autoName = autoChooser.getSelected();
        return buildAutoCommand(alliance, autoName);
    }

    // This is where autos actually run.
    // When you add a new PathPlanner auto, add a case here.
    private Command buildAutoCommand(String alliance, String autoName) {

        if (alliance.equals("RED")) {
            switch (autoName) {
                case "auto 1": return Commands.print("Running auto 1");
                case "auto 2": return Commands.print("Running auto 2");
                case "auto 3": return Commands.print("Running auto 3");
            }
        } else {
            switch (autoName) {
                case "auto 1": return Commands.print("Running auto 1");
                case "auto 2": return Commands.print("Running auto 2");
                case "auto blue 3": return Commands.print("Running auto blue 3");
            }
        }

        return Commands.print("No auto selected");
    }
}
