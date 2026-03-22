package frc.robot.autoCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class AutoIntakePulseCommand extends Command {

    private final Intake intake;
    private final Timer timer = new Timer();

    private static final double pulseTime = 0.25; //seconds
    private static final double pulseVolts = 2.0;   //small movement

    private boolean goingUp = true;

    public AutoIntakePulseCommand(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

     @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        if (timer.get() > pulseTime) {
            goingUp = !goingUp;
            timer.reset();
        }

        if (goingUp) {
            intake.setPivotVoltage(-pulseVolts);
        } else {
            intake.setPivotVoltage(pulseVolts); 
        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopPivot();
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return timer.get() >= 6;
    }
}
