package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterPresets;

public class FeedFromNeutral extends Command {

    private final Shooter shooter;

    public FeedFromNeutral(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setHoodAngle(ShooterPresets.NEUTRAL_ANGLE);
        shooter.setFlywheelRPM(ShooterPresets.NEUTRAL_RPM);
    }

    @Override
    public void execute() {
        if (shooter.readyToShoot()) {
            shooter.setFeederSpeed(ShooterPresets.NEUTRAL_RPM);
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.feedAndFlywheel(0);
    }

    @Override
    public boolean isFinished() {
        return false; // run while button held
    }
}
