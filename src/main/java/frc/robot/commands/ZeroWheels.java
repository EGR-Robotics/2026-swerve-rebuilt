package frc.robot.commands;

import edu.wpi.first.wpilibj.Preferences;
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
            double absolute = m.angleEncoder.getAbsolutePosition().getValueAsDouble();

            // Save offset to Preferences and static array
            String key = "Swerve/Offset" + m.corner;
            Preferences.setDouble(key, absolute);
            Module.STEER_OFFSETS[m.corner] = absolute;

            // Seed motor encoder from this new offset
            m.seedSteerFromAbsolute();

            System.out.println("Swerve module " + m.corner + " offset = " + absolute);
        }

        Preferences.setBoolean("Swerve/OffsetsSet", true);
        System.out.println("Swerve offsets saved. You should not need to re-zero again unless hardware changes.");
    }
}
