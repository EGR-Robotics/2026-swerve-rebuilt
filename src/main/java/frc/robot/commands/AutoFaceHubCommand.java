package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class AutoFaceHubCommand extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final Vision vision;
    private final PIDController turnPID = new PIDController(0.03, 0, 0);
    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric();

    public AutoFaceHubCommand(CommandSwerveDrivetrain drivetrain, Vision vision) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {

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
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(
            driveRequest
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0)
        );
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}