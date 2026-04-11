package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterPresets;

public class ByOutpostShot extends Command {

    private final Shooter shooter;

    public ByOutpostShot(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setHoodAngle(ShooterPresets.OUTPOST_ANGLE);
        shooter.setFlywheelRPM(ShooterPresets.OUTPOST_RPM);
    }

    @Override
    public void execute() {
        // if (shooter.readyToShoot()) {
        //     shooter.setFeederSpeed(ShooterPresets.NEUTRAL_RPM);
        // }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setFlywheelRPM(0);
        shooter.setHoodAngle(0);
        

    }

    @Override
    public boolean isFinished() {
        return false; // run while button held
    }
}
