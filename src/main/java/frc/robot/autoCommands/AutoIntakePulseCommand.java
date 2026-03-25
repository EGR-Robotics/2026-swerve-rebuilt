package frc.robot.autoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Intake;

public class AutoIntakePulseCommand extends Command {

    private final Intake intake;
    private final Command sequence;

    public AutoIntakePulseCommand(Intake intake) {
        this.intake = intake;

        // Build the safe, finite sequence
        this.sequence = Commands.sequence(
            // Up pulse
            // Commands.run(() -> intake.setPivotVoltage(-3.0), intake)
            //     .withTimeout(0.25),

            // // Down pulse
            // Commands.run(() -> intake.setPivotVoltage(3.0), intake)
            //     .withTimeout(0.25),

            // // Stop motors
            // Commands.runOnce(() -> intake.stopAll(), intake),

            // // Wait 3 seconds with motors OFF
            // Commands.waitSeconds(3.0)
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
    }

    @Override
    public void end(boolean interrupted) {
        sequence.end(interrupted);
        intake.stopAll();
    }

    @Override
    public boolean isFinished() {
        return sequence.isFinished();
    }
}
