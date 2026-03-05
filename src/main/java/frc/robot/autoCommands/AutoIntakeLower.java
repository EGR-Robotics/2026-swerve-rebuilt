package frc.robot.autoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class AutoIntakeLower extends Command {

    private final Intake intake;

    public AutoIntakeLower(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.lowerIntake();

        withTimeout(0.05);
    }

    @Override
    public void execute() {
        // if (shooter.readyToShoot()) {
        //     shooter.setFeederSpeed(ShooterPresets.HUB_RPM);
        // }
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopPivot();
    }

    @Override
    public boolean isFinished() {
        return false; // run while button held
    }
}
