package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.ShooterPresets;

public class FeedFromOpposite extends Command {

    private final Shooter shooter;

    public FeedFromOpposite(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setHoodAngle(ShooterPresets.OPPOSITE_ANGLE);
        shooter.setFlywheelRPM(ShooterPresets.OPPOSITE_RPM);
    }

    @Override
    public void execute() {
        if (shooter.readyToShoot()) {
            shooter.setFeederSpeed(ShooterPresets.OPPOSITE_RPM);
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.feedAndFlywheel(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
