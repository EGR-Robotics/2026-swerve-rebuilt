package frc.robot.autoCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterPresets;

public class Auto_ByOutpostShot extends Command {

    private final Shooter shooter;
    private Timer timer;

    public Auto_ByOutpostShot(Shooter shooter) {
        this.shooter = shooter;
        this.timer = new Timer();

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setHoodAngle(ShooterPresets.OUTPOST_ANGLE);
        shooter.setFlywheelRPM(ShooterPresets.OUTPOST_RPM);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        
        shooter.setFlywheelRPM(0);
        shooter.setHoodAngle(5);
    }

    @Override
    public boolean isFinished() {
        return timer.get() >= 9;
    }
}
