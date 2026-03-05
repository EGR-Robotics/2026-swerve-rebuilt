package frc.robot.autoCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter.Shooter;

public class AutoTuneShot extends Command {

    private final Shooter shooter;
    private final CommandSwerveDrivetrain drivetrain;

    // Shooter offset from robot center (meters)
    private static final Translation2d SHOOTER_OFFSET = new Translation2d(0.3, 0.0);

    // Target location on field (hub)
    private static final Translation2d TARGET_POS = new Translation2d(0, 1.829);
    public AutoTuneShot(Shooter shooter, CommandSwerveDrivetrain drivetrain) {
        this.shooter = shooter;
        this.drivetrain = drivetrain;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        SmartDashboard.putNumber("TUNE RPM", 3000);
        SmartDashboard.putNumber("TUNE ANGLE", 60);
    }

    @Override
    public void execute() {

        // -------------------------
        // Distance Calculation
        // -------------------------
        Pose2d robotPose = drivetrain.getState().Pose;
        Translation2d shooterPos = robotPose.getTranslation().plus(SHOOTER_OFFSET);

        double distance = shooterPos.getDistance(TARGET_POS);
        SmartDashboard.putNumber("Shot Distance", distance);

        // -------------------------
        // Manual Tuning
        // -------------------------
        double rpm = SmartDashboard.getNumber("TUNE RPM", 0);
        double angle = SmartDashboard.getNumber("TUNE ANGLE", 60);

        shooter.setFlywheelRPM(rpm);
        shooter.setHoodAngle(angle);

        // -------------------------
        // Logging
        // -------------------------
        SmartDashboard.putNumber("Current RPM", shooter.getFlywheelRPM());
        SmartDashboard.putNumber("Current Angle", shooter.getHoodAngle());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
