package frc.robot.autoCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class Auto_AutoFaceHubCommand extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final Vision vision;

    // kP is a starting point – you can bump it up/down after testing
    private final PIDController turnPID = new PIDController(1, 0, 0.15);

    // Robot-relative rotation so the robot actually spins
    private final SwerveRequest.RobotCentric driveRequest =
        new SwerveRequest.RobotCentric();

    // How close to center (in degrees of tx) we consider "good enough"
    private static final double YAW_TOLERANCE_DEG = 1.5;

    public Auto_AutoFaceHubCommand(CommandSwerveDrivetrain drivetrain, Vision vision) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {

        double yawError = vision.getYawToHub(); // +right, -left from your priority list
        double rotCmd = 0.0;

        if (!Double.isNaN(yawError)) {

            // We want error = yawError (not -yawError)
            // PID form: calculate(measurement, setpoint)
            // So: measurement = 0, setpoint = yawError → error = yawError - 0 = yawError
            rotCmd = turnPID.calculate(0.0, yawError);

            // Limelight: +tx = target right
            // Phoenix: +rot = CCW (left)
            // To turn TOWARD the target, we need to flip the sign: 
            rotCmd = -rotCmd; //todo: flip this sign if it rotates the wrong direction. 

            // Optional: deadband near center so it doesn't hunt forever
            if (Math.abs(yawError) < YAW_TOLERANCE_DEG) {
                rotCmd = 0.0;
            }
        }

        // Clamp to something reasonable; you can tune this
        rotCmd = MathUtil.clamp(rotCmd, -2.0, 2.0);

        drivetrain.setControl(
            driveRequest
                .withVelocityX(0)      // you can later replace these with driver translation
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
