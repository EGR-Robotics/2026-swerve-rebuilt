package frc.robot.autoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feed;

public class AutoFeed extends Command {

    private final Feed feed;

    public AutoFeed(Feed feed) {
        this.feed = feed;
        addRequirements(feed);
    }

    @Override
    public void initialize() {
        feed.setRollerRPM(9000);
        feed.setFeederSpeed(9000);

        withTimeout(3);
    }

    @Override
    public void execute() {
        // if (shooter.readyToShoot()) {
        //     shooter.setFeederSpeed(ShooterPresets.HUB_RPM);
        // }
    }

    @Override
    public void end(boolean interrupted) {
        feed.stopFeed(0);
        feed.stopRoller();
    }

    @Override
    public boolean isFinished() {
        return false; // run while button held
    }
}
