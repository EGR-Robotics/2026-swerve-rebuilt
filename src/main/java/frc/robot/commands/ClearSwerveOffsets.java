package frc.robot.commands;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ClearSwerveOffsets extends InstantCommand {

    @Override
    public void initialize() {

        // Clear each module's offset
        for (int i = 0; i < 4; i++) {
            Preferences.setDouble("Swerve/Offset" + i, 0.0);
        }

        // Mark offsets as not set
        Preferences.setBoolean("Swerve/OffsetsSet", false);

        System.out.println("Swerve offsets cleared. Reboot robot.");
    }
}
