package frc.robot.autoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feed;
import frc.robot.subsystems.Intake;

public class AutoIntake extends Command {

    private final Intake intake;

    public AutoIntake(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.intake(1);

        withTimeout(5);
    }

    @Override
    public void execute() {
        // if (shooter.readyToShoot()) {
        //     shooter.setFeederSpeed(ShooterPresets.HUB_RPM);
        // }
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopRoller();
    }

    @Override
    public boolean isFinished() {
        return false; // run while button held
    }
}
