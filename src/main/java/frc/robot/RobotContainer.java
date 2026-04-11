package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.List;
import java.util.HashMap;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.autoCommands.AutoClimbDown;
import frc.robot.autoCommands.AutoClimbUp;
import frc.robot.autoCommands.AutoFeed;
import frc.robot.autoCommands.AutoFeedFromNeutral;
import frc.robot.autoCommands.AutoHubShot;
import frc.robot.autoCommands.AutoIntake;
import frc.robot.autoCommands.AutoIntakeLower;
import frc.robot.autoCommands.AutoIntakePulseCommand;
import frc.robot.autoCommands.AutoIntakeRaise;
import frc.robot.autoCommands.AutoTrenchShot;
import frc.robot.autoCommands.Auto_ByOutpostShot;
import frc.robot.commands.ByOutpostShot;
import frc.robot.commands.FeedFromNeutral;
import frc.robot.commands.FeedFromOpposite;
import frc.robot.commands.IntakePulseCommand;
import frc.robot.commands.Hub2Preset;
import frc.robot.commands.TrenchShot;
import frc.robot.commands.UnderClimbShot;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Feed;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakePivot;
// import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterTable;

public class RobotContainer {

    private double MaxSpeed = 0.6 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double SlowSpeed = 0.1 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
    private double MAX_RPM = 3500;

    private final SlewRateLimiter xLimiter = new SlewRateLimiter(2);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(2);
    private final SlewRateLimiter rotLimiter = new SlewRateLimiter(2);

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
    public IntakePivot intakepivot = new IntakePivot();
    // public Vision vision = new Vision();
    public ShooterTable table = new ShooterTable();

    private SendableChooser<String> allianceChooser = new SendableChooser<>();
    private SendableChooser<String> autoChooser = new SendableChooser<>();

    private final List<String> redAutos = List.of(
        "ChocolateChip",
        "WhiteChocolateChip",
        "MandMCookie",
        "OatmealRaisin",
        "DoubleChocolate",
        "Oreo",
        "ChipsAhoy",
        "NutterButter",
        "SugarCookie",
        "Gingerbread",
        "ThinMint",
        "TESTCOOKIE"
    );

    private final List<String> blueAutos = List.of(
        "ChocolateChip",
        "WhiteChocolateChip",
        "MandMCookie",
        "OatmealRaisin",
        "DoubleChocolate",
        "Oreo",
        "FortuneCookie",
        "ChipsAhoy",
        "NutterButter"
    );

    // ⭐ NEW: Prebuilt autos stored here
    private final HashMap<String, Command> builtAutos = new HashMap<>();

    public RobotContainer() {
        configureBindings();
        configureAutoSelectors();

        NamedCommands.registerCommand("AutoIntakeLower", new AutoIntakeLower(intakepivot));
        NamedCommands.registerCommand("AutoIntake", new AutoIntake(intake));
        NamedCommands.registerCommand("AutoHubShot", new AutoHubShot(shooter));
        NamedCommands.registerCommand("Auto_ByOutpostShot", new Auto_ByOutpostShot(shooter));
        NamedCommands.registerCommand("AutoFeedFromNeutral", new AutoFeedFromNeutral(shooter));
        NamedCommands.registerCommand("AutoFeed", new AutoFeed(feed));
        NamedCommands.registerCommand("AutoTrenchShot", new AutoTrenchShot(shooter));
        NamedCommands.registerCommand("AutoIntakeRaise", new AutoIntakeRaise(intakepivot));
        NamedCommands.registerCommand("AutoIntakePulseCommand", new AutoIntakePulseCommand(intakepivot));
        NamedCommands.registerCommand("AutoClimbUp", new AutoClimbUp(climber));
        NamedCommands.registerCommand("AutoClimbDown", new AutoClimbDown(climber));

        drivetrain.configureAutoBuilder();

        // ⭐ NEW: Build all autos at startup
        buildAllAutos();
    }

    // ⭐ NEW: Prebuild all autos once
    private void buildAllAutos() {
        for (String name : redAutos) {
            try {
                builtAutos.put("RED_" + name, buildAutoCommand("RED", name));
            } catch (Exception e) {
                System.err.println("Failed to build RED auto: " + name);
                e.printStackTrace();
            }
        }

        for (String name : blueAutos) {
            try {
                builtAutos.put("BLUE_" + name, buildAutoCommand("BLUE", name));
            } catch (Exception e) {
                System.err.println("Failed to build BLUE auto: " + name);
                e.printStackTrace();
            }
        }

        System.out.println("Finished attempting to build all autos.");
    }


    private void configureAutoSelectors() {
        allianceChooser.setDefaultOption("Red Alliance", "RED");
        allianceChooser.addOption("Blue Alliance", "BLUE");
        SmartDashboard.putData("Alliance Selector", allianceChooser);

        populateAutoChooser("RED");
        SmartDashboard.putData("Auto Selector", autoChooser);

        allianceChooser.onChange(alliance -> populateAutoChooser(alliance));
    }

    private void populateAutoChooser(String alliance) {
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

                if (driverController.x().getAsBoolean()) {
                    return drive.withVelocityX(limitedX * SlowSpeed)
                            .withVelocityY(limitedY * SlowSpeed)
                            .withRotationalRate(limitedRot * MaxAngularRate);

                } else {
                    return drive.withVelocityX(limitedX * MaxSpeed)
                            .withVelocityY(limitedY * MaxSpeed)
                            .withRotationalRate(limitedRot * MaxAngularRate);
                }
            })
        );

        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        driverController.leftTrigger().whileTrue(
            new RunCommand(() -> intake.intake(), intake)
                .finallyDo(() -> intake.stop())
        );

        driverController.leftBumper().whileTrue(
            new RunCommand(() -> intakepivot.lowerIntake(), intakepivot)
        ).onFalse(
            new InstantCommand(() -> intakepivot.stop(), intakepivot)
        );

        driverController.rightTrigger().whileTrue(
            new RunCommand(() -> intake.reverse(), intake)
                .finallyDo(() -> intake.stop())
        );

        driverController.rightBumper().whileTrue(
            new RunCommand(() -> intakepivot.raiseIntake(), intakepivot)
        ).onFalse(
            new InstantCommand(() -> intakepivot.stop(), intakepivot)
        );

        driverController.y().whileTrue(
            new RunCommand(() -> feed.reverseRoller(6000), feed)
        ).onFalse(
            new InstantCommand(() -> feed.stopRoller(), feed)
        );

        driverController.povUp().onTrue(
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric())
        );

        driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        operatorController.leftTrigger().whileTrue(
            new RunCommand(() -> {
                double flywheelTrigger = operatorController.getLeftTriggerAxis();
                double rpm = flywheelTrigger * MAX_RPM;
                shooter.setFlywheelRPM(rpm);
            }, shooter).finallyDo(() -> shooter.setFlywheelRPM(0))
        );

        operatorController.rightTrigger().whileTrue(
            Commands.parallel(
                Commands.startEnd(
                    () -> feed.feedFuel(),
                    () -> feed.stopFeed(0),
                    feed),
                new IntakePulseCommand(intakepivot))
        );

        operatorController.leftBumper().whileTrue(new AutoClimbUp(climber));
        operatorController.rightBumper().whileTrue(new AutoClimbDown(climber));

        operatorController.y().whileTrue(new ByOutpostShot(shooter));
        operatorController.x().whileTrue(new TrenchShot(shooter));
        operatorController.b().whileTrue(new UnderClimbShot(shooter));
        operatorController.a().whileTrue(new Hub2Preset(shooter));

        operatorController.povRight().whileTrue(new FeedFromNeutral(shooter));
        operatorController.povLeft().whileTrue(new FeedFromOpposite(shooter));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    // ⭐ UPDATED: Now returns prebuilt autos instantly
    public Command getAutonomousCommand() {
        String alliance = allianceChooser.getSelected();
        String autoName = autoChooser.getSelected();

        if (alliance == null) alliance = "RED";
        if (autoName == null) return Commands.print("No auto selected (autoName was null)");

        return builtAutos.get(alliance + "_" + autoName);
    }

    private Command buildAutoCommand(String alliance, String autoName) {

        if ("RED".equals(alliance)) {
            switch (autoName) {
                case "ChocolateChip": return new PathPlannerAuto("RedCenterToCenter");
                case "WhiteChocolateChip": return new PathPlannerAuto("RedCenterToCenterToClimb");
                case "MandMCookie": return new PathPlannerAuto("RedCenterToCenterToOutpostToCenter");
                case "OatmealRaisin": return new PathPlannerAuto("RedCenterToCenterToOutpost");
                case "DoubleChocolate": return new PathPlannerAuto("RedCenterToCenterToOutpostToCenterToClimb");
                case "Oreo": return new PathPlannerAuto("RedLeftTrenchToNeutralStartToNeutralMidToLeftTrench");
                case "ChipsAhoy": return new PathPlannerAuto("RedRightTrenchToNeutralStartToNeutralMidToRightTrench");
                case "NutterButter": return new PathPlannerAuto("RedRightTrenchToNeutralStartToNeutralMidToRightTrenchToOutpostToOutpostShoot");
                case "SugarCookie": return new PathPlannerAuto("RedLeftTrenchToNeutralStartToNeutralMidFeedToLeftTrenchCurve");
                case "Gingerbread": return new PathPlannerAuto("RedLeftTrenchNeutralMidToLeftTrenchCurve");
                case "ThinMint": return new PathPlannerAuto("RedRightBumpToOutpostToClimb");
                case "TESTCOOKIE": return new PathPlannerAuto("RedClimb");
            }
        } else {
            switch (autoName) {
                case "ChocolateChip": return new PathPlannerAuto("BlueCenterToCenter");
                case "WhiteChocolateChip": return new PathPlannerAuto("BlueCenterToCenterToClimb");
                case "MandMCookie": return new PathPlannerAuto("BlueCenterToCenterToOutpostToCenter");
                case "OatmealRaisin": return new PathPlannerAuto("BlueCenterToCenterToOutpost");
                case "DoubleChocolate": return new PathPlannerAuto("BlueCenterToCenterToOutpostToCenterToClimb");
                case "FortuneCookie": return new PathPlannerAuto("BlueLeftTrenchChaosToLeftTrench");
                case "Oreo": return new PathPlannerAuto("BlueLeftTrenchToNeutralStartToNeutralMidToLeftTrench");
                case "ChipsAhoy": return new PathPlannerAuto("BlueRightTrenchToNeutralStartToNeutralMidToRightTrench");
                case "NutterButter": return new PathPlannerAuto("BlueRightTrenchToNeutralStartToNeutralMidToRightTrenchToOutpostToOutpostShoot");
            }
        }

        return Commands.print("No auto selected");
    }
}
