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

    private final PIDController turnPID = new PIDController(0.03, 0, 0);
    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric();

    // Option 3: last known good + odometry delta correction
    private double lastKnownDist = Double.NaN;
    private Pose2d poseAtLastLock = null;

    public AutoShooterAlignCommand(CommandSwerveDrivetrain drivetrain, Vision vision, Shooter shooter, ShooterTable table) {
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

        // --- Swerve rotation toward hub ---
        double yawError = vision.getYawToHub();
        double rotCmd = 0.0;

        if (!Double.isNaN(yawError)) {
            rotCmd = turnPID.calculate(yawError, 0);
        }

        rotCmd = MathUtil.clamp(rotCmd, -2.0, 2.0);

        drivetrain.setControl(
            driveRequest
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(rotCmd)
        );

        // --- Shooter aiming ---
        Pose2d currentPose = drivetrain.getState().Pose;
        double dist = vision.getDistanceToHub();

        if (!Double.isNaN(dist)) {
            // Fresh vision lock — update snapshot
            lastKnownDist = dist;
            poseAtLastLock = currentPose;
        } else if (!Double.isNaN(lastKnownDist) && poseAtLastLock != null) {
            // Vision lost — estimate how far we've moved since last lock
            // and correct the last known distance by that delta
            double odometryDelta = currentPose.getTranslation()
                .getDistance(poseAtLastLock.getTranslation());
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