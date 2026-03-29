package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Intake;

public class IntakePulseCommand extends Command {

    private final Intake intake;
    private final Command sequence;

    public IntakePulseCommand(Intake intake) {
        this.intake = intake;

        this.sequence = Commands.sequence(
            Commands.run(() -> intake.raiseIntake(), intake)
                .withTimeout(0.2),

            Commands.run(() -> intake.lowerIntake(), intake)
                .withTimeout(0.05),

            Commands.runOnce(() -> intake.stopAll(), intake),

            Commands.waitSeconds(0.2)
        );

        addRequirements(intake);
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
        intake.stopAll();
    }

    @Override
    public boolean isFinished() {
        return false; // runs forever until button released
    }
}
