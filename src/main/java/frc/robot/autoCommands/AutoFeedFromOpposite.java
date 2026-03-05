package frc.robot.autoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterPresets;

public class AutoFeedFromOpposite extends Command {

    private final Shooter shooter;

    public AutoFeedFromOpposite(Shooter shooter) {
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
        // if (shooter.readyToShoot()) {
        //     shooter.setFeederSpeed(ShooterPresets.OPPOSITE_RPM);
        // }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setFlywheelRPM(0);
        shooter.setHoodAngle(5);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
