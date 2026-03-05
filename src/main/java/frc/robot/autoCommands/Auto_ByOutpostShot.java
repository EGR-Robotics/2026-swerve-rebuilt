package frc.robot.autoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterPresets;

public class Auto_ByOutpostShot extends Command {

    private final Shooter shooter;

    public Auto_ByOutpostShot(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setHoodAngle(ShooterPresets.OUTPOST_ANGLE);
        shooter.setFlywheelRPM(ShooterPresets.OUTPOST_RPM);

        withTimeout(5);
        // shooter.setHoodAngle(ShooterPresets.TESTING_ANGLE);
        // shooter.setFlywheelRPM(ShooterPresets.TESTING_RPM);
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
