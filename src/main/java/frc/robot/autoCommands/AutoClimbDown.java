package frc.robot.autoCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class AutoClimbDown extends Command {

    private final Climber climber;
    private Timer timer;

    public AutoClimbDown(Climber climber) {
        this.climber = climber;
        this.timer = new Timer();

        addRequirements(climber);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();

        climber.autoClimbDown();
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

        climber.stop();
    }

    @Override
    public boolean isFinished() {
        return timer.get() >= .25;
    }
}
