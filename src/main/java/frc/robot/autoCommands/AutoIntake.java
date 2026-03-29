package frc.robot.autoCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feed;
import frc.robot.subsystems.Intake;

public class AutoIntake extends Command {

    private final Intake intake;
    private Timer timer;

    public AutoIntake(Intake intake) {
        this.intake = intake;
        this.timer = new Timer();

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();

        intake.intake();
    }

    @Override
    public void execute() {
        // if (shooter.readyToShoot()) {
        //     shooter.setFeederSpeed(ShooterPresets.HUB_RPM);
        // }
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();

        intake.stopRoller();
    }

    @Override
    public boolean isFinished() {
        return timer.get() >= 2;
    }
}
