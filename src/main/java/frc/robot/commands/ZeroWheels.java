package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Module;

public class ZeroWheels extends InstantCommand {

    private final Module[] modules;

    public ZeroWheels(Module... modules) {
        this.modules = modules;
    }

    @Override
    public void initialize() {
        for (Module m : modules) {
            m.zeroSteerToForward();
            System.out.println("Module " + m.corner + " steer zeroed to forward.");
        }
    }
}
