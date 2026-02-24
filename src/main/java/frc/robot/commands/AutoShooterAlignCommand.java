package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.ShooterTable;

public class AutoShooterAlignCommand extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final Shooter shooter;
    private final ShooterTable table;

    public AutoShooterAlignCommand(CommandSwerveDrivetrain drivetrain, Shooter shooter, ShooterTable table) {
        this.drivetrain = drivetrain;
        this.shooter = shooter;
        this.table = table;
        addRequirements(shooter);
    }

    @Override
    public void execute() {

        double dist = drivetrain.getDistanceToHub();

        if (Double.isNaN(dist)) {
            // No vision → do nothing
            return;
        }

        ShooterTable.ShooterSetpoint set = table.getSetpoint(dist);

        shooter.setHoodAngle(set.angle());
        shooter.setFlywheelRPM(set.rpm());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
