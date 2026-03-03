package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterTable;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class AutoShooterAlignCommand extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final Vision vision;
    private final Shooter shooter;
    private final ShooterTable table;

    // Rotation PID (robot-relative turning)
    private final PIDController turnPID = new PIDController(0.03, 0, 0);

    // Robot-relative request (your Phoenix version supports this)
    private final SwerveRequest.RobotCentric driveRequest =
        new SwerveRequest.RobotCentric();

    // Distance fallback
    private double lastKnownDist = Double.NaN;
    private Pose2d poseAtLastLock = null;

    // How close to center we consider "aligned"
    private static final double YAW_TOLERANCE_DEG = 1.0;

    public AutoShooterAlignCommand(
        CommandSwerveDrivetrain drivetrain,
        Vision vision,
        Shooter shooter,
        ShooterTable table
    ) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.shooter = shooter;
        this.table = table;
        addRequirements(drivetrain, shooter);
    }

    @Override
    public void initialize() {
        lastKnownDist = Double.NaN;
        poseAtLastLock = null;
    }

    @Override
    public void execute() {

        // rotates robot
        double yawError = vision.getYawToHub();   // +right, -left
        double rotCmd = 0.0;

        if (!Double.isNaN(yawError)) {

            // PID form: calculate(measurement, setpoint)
            // We want error = yawError, so:
            rotCmd = turnPID.calculate(0.0, yawError);

            // Limelight: +tx = target right
            // Robot: +rot = CCW (left)
            // To turn TOWARD the target:
            rotCmd = -rotCmd;

            // Stop hunting when nearly centered
            if (Math.abs(yawError) < YAW_TOLERANCE_DEG) {
                rotCmd = 0.0;
            }
        }

        rotCmd = MathUtil.clamp(rotCmd, -2.0, 2.0);

        drivetrain.setControl(
            driveRequest
                .withVelocityX(0)      // can later replace with driver translation
                .withVelocityY(0)
                .withRotationalRate(rotCmd)
        );

        // hood + flywheel
        Pose2d currentPose = drivetrain.getState().Pose;
        double dist = vision.getDistanceToHub();

        if (!Double.isNaN(dist)) {
            // Fresh lock
            lastKnownDist = dist;
            poseAtLastLock = currentPose;
        } else if (!Double.isNaN(lastKnownDist) && poseAtLastLock != null) {
            // Vision lost → estimate distance change
            double odometryDelta =
                currentPose.getTranslation().getDistance(
                    poseAtLastLock.getTranslation()
                );
            dist = lastKnownDist + odometryDelta;
        }

        if (!Double.isNaN(dist)) {
            ShooterTable.ShooterSetpoint set = table.getSetpoint(dist);
            shooter.setHoodAngle(set.angle());
            shooter.setFlywheelRPM(set.rpm());
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(
            driveRequest
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0)
        );
        shooter.setFlywheelRPM(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
