package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class AutoFaceHubCommand extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final PIDController turnPID = new PIDController(0.03, 0, 0);

    public AutoFaceHubCommand(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {

        double yawError = drivetrain.getYawToHub();
        double rotCmd = 0.0;

        if (!Double.isNaN(yawError)) {
            rotCmd = turnPID.calculate(yawError, 0);
        }

        rotCmd = MathUtil.clamp(rotCmd, -2.0, 2.0);

        drivetrain.setControl(
            new SwerveRequest.FieldCentric()
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(rotCmd)
        );
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(
            new SwerveRequest.FieldCentric()
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
