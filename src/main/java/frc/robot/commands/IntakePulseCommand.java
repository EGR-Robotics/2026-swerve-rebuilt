package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakePivot;

public class IntakePulseCommand extends Command {

    private final IntakePivot intakepivot;
    private final Command sequence;

    public IntakePulseCommand(IntakePivot intakepivot) {
        this.intakepivot = intakepivot;

        this.sequence = Commands.sequence(
            Commands.run(() -> intakepivot.raiseIntake(), intakepivot).withTimeout(0.20),

            Commands.run(() -> intakepivot.lowerIntake(), intakepivot).withTimeout(0.12),

            Commands.runOnce(() -> intakepivot.stop(), intakepivot),

            Commands.waitSeconds(0.4)
        );

        addRequirements(intakepivot);
    }

    @Override
    public void initialize() {
        sequence.initialize();
    }

    @Override
    public void execute() {
        sequence.execute();

        // Restart the sequence when it finishes
        if (sequence.isFinished()) {
            sequence.end(false);
            sequence.initialize();
        }
    }

    @Override
    public void end(boolean interrupted) {
        sequence.end(interrupted);
        intakepivot.stop();
    }

    @Override
    public boolean isFinished() {
        return false; // runs forever until button released
    }
}
