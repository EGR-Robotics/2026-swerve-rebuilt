package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;

public class AutoAngleAlign extends Command {
    private CommandSwerveDrivetrain drivetrain;
    private Vision vision;

    public AutoAngleAlign(CommandSwerveDrivetrain drivetrain, Vision vision) {
        this.drivetrain = drivetrain;
        this.vision = vision;
    }

    @Override
    public void initialize(){

    }

    @Override 
    public void execute() {
        
    }
    
    @Override
    public boolean isFinished(){
        return false;
    }
}
