package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterPresets;

public class AgainstHubShot extends Command {

    private final Shooter shooter;

    public AgainstHubShot(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setHoodAngle(ShooterPresets.HUB_ANGLE);
        shooter.setFlywheelRPM(ShooterPresets.HUB_RPM);
    }

    @Override
    public void execute() {
        // if (shooter.readyToShoot()) {
        //     shooter.setFeederSpeed(ShooterPresets.HUB_RPM);
        // }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setFlywheelRPM(0);
    }

    @Override
    public boolean isFinished() {
        return false; // run while button held
    }
}
