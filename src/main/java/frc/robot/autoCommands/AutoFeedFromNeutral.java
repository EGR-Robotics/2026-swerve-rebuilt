package frc.robot.autoCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterPresets;

public class AutoFeedFromNeutral extends Command {

    private final Shooter shooter;
    private Timer timer;

    public AutoFeedFromNeutral(Shooter shooter) {
        this.shooter = shooter;
        this.timer = new Timer();

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();

        shooter.setHoodAngle(ShooterPresets.NEUTRAL_ANGLE);
        shooter.setFlywheelRPM(ShooterPresets.NEUTRAL_RPM);

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

        shooter.setFlywheelRPM(0);
        shooter.setHoodAngle(5);
    }

    @Override
    public boolean isFinished() {
        return timer.get() >= 8;
    }
}
