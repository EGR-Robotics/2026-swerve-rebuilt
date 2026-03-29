package frc.robot.autoCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakePivot;

public class AutoIntakePulseCommand extends Command {

    private final IntakePivot intakepivot;
    private Command sequence;
    private final Timer timer = new Timer();

    public AutoIntakePulseCommand(IntakePivot intakepivot) {
        this.intakepivot = intakepivot;

        addRequirements(intakepivot);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();

        // Rebuild a fresh sequence each time (prevents WPILib index errors)
        sequence = Commands.sequence(
            Commands.run(() -> intakepivot.raiseIntake(), intakepivot).withTimeout(0.35),
            Commands.run(() -> intakepivot.lowerIntake(), intakepivot).withTimeout(0.05),
            Commands.runOnce(() -> intakepivot.stop(), intakepivot),
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
                Commands.run(() -> intakepivot.raiseIntake(), intakepivot).withTimeout(0.35),
                Commands.run(() -> intakepivot.lowerIntake(), intakepivot).withTimeout(0.05),
                Commands.runOnce(() -> intakepivot.stop(), intakepivot),
                Commands.waitSeconds(0.2)
            );

            sequence.initialize();
        }
    }

    @Override
    public void end(boolean interrupted) {
        sequence.end(interrupted);
        intakepivot.stop();
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(2.0); // ends after 2 seconds
    }
}
