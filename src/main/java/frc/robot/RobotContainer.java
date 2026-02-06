package frc.robot;

import static frc.robot.Constants.ANGLE_CLOSE_RAD;
import static frc.robot.Constants.MAX_ANGULAR_SPEED;
import static frc.robot.Constants.MAX_LINEAR_SPEED_TELEOP;
import static frc.robot.Constants.normConstraints;
import static frc.robot.Utils.getSpeed2;

import java.util.Set;
import java.util.concurrent.Future;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.commands.ZeroWheels;
import frc.robot.subsystems.Drive;

public class RobotContainer {

    

    private final Drive drive = new Drive();
    private final CommandXboxController m_driverController = new CommandXboxController(0);

    private Future<PathPlannerPath> onTheFlyPath = null;

    public RobotContainer() {

        // Load persistent offsets (NO ZeroWheels at startup)
        boolean offsetsSet = Preferences.getBoolean("Swerve/OffsetsSet", false);

        if (offsetsSet) {
            drive.frontLeftModule.seedSteerFromAbsolute();
            drive.frontRightModule.seedSteerFromAbsolute();
            drive.backLeftModule.seedSteerFromAbsolute();
            drive.backRightModule.seedSteerFromAbsolute();
            System.out.println("Swerve offsets loaded from Preferences.");
        } else {
            System.out.println("Swerve offsets NOT set. Run ZeroWheels once with wheels straight to calibrate.");
        }

        // Warm up path planner (your original code)
        System.out.println("Warming up path planner");
        for (int i = 0; i < 10; i++) {
            var finalPathPoint = new Pose2d(
                    new Translation2d(Math.random() * 10 - 5, Math.random() * 10 - 5),
                    Rotation2d.fromDegrees(Math.random() * 360)
            );

            Rotation2d targetRotation = Rotation2d.fromDegrees(Math.random() * 360);
            var time = Timer.getFPGATimestamp();
            var path = AsyncPathGenerator.generatePathAsync(finalPathPoint, targetRotation, drive, normConstraints);
            try {
                var points = path.get().getAllPathPoints();
                var duration = Timer.getFPGATimestamp() - time;
                System.out.println("Generated path " + (i + 1) + "/10. " + points.size() + " points in " + duration + " seconds");
            } catch (Exception e) {
                System.out.println("Path failed to generate" + e);
            }
        }

        configureBindings();
    }

    private void configureBindings() {

        // X BUTTON → Run ZeroWheels manually
        //m_driverController.x().onTrue(
                //new ZeroWheels(
                       // drive.frontLeftModule,
                      //  drive.frontRightModule,
                       // drive.backLeftModule,
                       // drive.backRightModule
               // )
       // );

        // A button → hold heading test (your original)
        m_driverController.a().whileTrue(Commands.run(() -> drive.holdHeadingTest(), drive));

        // Default drive command
        drive.setDefaultCommand(
                new RunCommand(() -> {
                    var thetaInput = Math.abs(m_driverController.getRightX()) * m_driverController.getRightX() * MAX_ANGULAR_SPEED * -1;
                    var controls = getControls(m_driverController);
                    if (drive.shouldBumpAdjust()) {
                        drive.rotationPidDrive(controls.getX(), controls.getY(), drive.closestBumpAngle(), 0.0, 0.0);
                    } else {
                        drive.drive(controls.getX(), controls.getY(), thetaInput, true);
                    }
                }, drive));

        // Start button → zero pose
        m_driverController.start().onTrue(Commands.runOnce(drive::zeroPose, drive));
    }

    public static Translation2d getControls(CommandXboxController m_driverController) {
        var xInput = -Math.abs(m_driverController.getLeftY()) * m_driverController.getLeftY() * MAX_LINEAR_SPEED_TELEOP;
        var yInput = -Math.abs(m_driverController.getLeftX()) * m_driverController.getLeftX() * MAX_LINEAR_SPEED_TELEOP;
        return new Translation2d(xInput, yInput);
    }

    public Command getAutonomousCommand() {
        return null;
    }

    public void disabledInit() {
        CommandScheduler.getInstance().schedule(
                Commands.runOnce(() -> drive.setBrakeMode(NeutralModeValue.Brake))
                        .beforeStarting(Commands.waitSeconds(3.0))
                        .ignoringDisable(true)
        );
    }

    public void enabledInit() {
        CommandScheduler.getInstance().schedule(
                Commands.runOnce(() -> drive.setBrakeMode(NeutralModeValue.Brake))
        );
    }

    
}

