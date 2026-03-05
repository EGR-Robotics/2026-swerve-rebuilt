package frc.robot.autoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterPresets;

public class AutoHubShot extends Command {

    private final Shooter shooter;

    public AutoHubShot(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setHoodAngle(ShooterPresets.HUB_ANGLE);
        shooter.setFlywheelRPM(ShooterPresets.HUB_RPM);

        withTimeout(4);
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
        shooter.setHoodAngle(5);
    }

    @Override
    public boolean isFinished() {
        return false; // run while button held
    }
}
