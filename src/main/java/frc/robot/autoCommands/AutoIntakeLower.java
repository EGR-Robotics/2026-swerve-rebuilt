package frc.robot.autoCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakePivot;

public class AutoIntakeLower extends Command {

    private final IntakePivot intakepivot;
    private Timer timer;

    public AutoIntakeLower(IntakePivot intakepivot) {
        this.intakepivot = intakepivot;
        this.timer = new Timer();

        addRequirements(intakepivot);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();

        // intakepivot.lowerIntake();
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

        intakepivot.stop();
    }

    @Override
    public boolean isFinished() {
        return timer.get() >= 0.52;
    }
}
