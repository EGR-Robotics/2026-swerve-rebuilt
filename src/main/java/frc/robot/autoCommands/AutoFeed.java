package frc.robot.autoCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feed;

public class AutoFeed extends Command {

    private final Feed feed;
    private Timer timer;

    public AutoFeed(Feed feed) {
        this.feed = feed;
        this.timer = new Timer();

        addRequirements(feed);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();

        feed.setRollerRPM(9000);
        feed.setFeederSpeed(9000);
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
        
        feed.stopFeed(0);
        feed.stopRoller();
    }

    @Override
    public boolean isFinished() {
        return timer.get() >= 4;
    }
}
