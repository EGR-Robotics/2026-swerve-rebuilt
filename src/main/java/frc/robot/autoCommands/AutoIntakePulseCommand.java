package frc.robot.autoCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Intake;

public class AutoIntakePulseCommand extends Command {

    private final Intake intake;
    private Command sequence;
    private final Timer timer = new Timer();

    public AutoIntakePulseCommand(Intake intake) {
        this.intake = intake;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();

        // Rebuild a fresh sequence each time (prevents WPILib index errors)
        sequence = Commands.sequence(
            Commands.run(() -> intake.raiseIntake(), intake).withTimeout(0.35),
            Commands.run(() -> intake.lowerIntake(), intake).withTimeout(0.05),
            Commands.runOnce(() -> intake.stopAll(), intake),
            Commands.waitSeconds(0.2)
        );

        sequence.initialize();
    }

    @Override
    public void execute() {
        sequence.execute();

        // Restart the sequence cleanly when it finishes
        if (sequence.isFinished()) {
            sequence.end(false);

            sequence = Commands.sequence(
                Commands.run(() -> intake.raiseIntake(), intake).withTimeout(0.35),
                Commands.run(() -> intake.lowerIntake(), intake).withTimeout(0.05),
                Commands.runOnce(() -> intake.stopAll(), intake),
                Commands.waitSeconds(0.2)
            );

            sequence.initialize();
        }
    }

    @Override
    public void end(boolean interrupted) {
        sequence.end(interrupted);
        intake.stopAll();
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(2.0); // ends after 2 seconds
    }
}
