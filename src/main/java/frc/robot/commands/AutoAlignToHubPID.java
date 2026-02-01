package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.vision.VisionPositionValues;
import java.util.function.DoubleSupplier;

/**
 * PID-based auto alignment to the hub using Limelight tx.
 *
 * This version allows:
 *  - Driving normally (translation from joysticks)
 *  - Holding RB to continuously auto-align rotation
 *  - Aligning while stationary if needed
 *
 * HOW TO TUNE THE PID:
 * ---------------------
 * 1. Start with kP only (0.02–0.05 is typical)
 *    - Increase kP until the robot turns quickly toward the target
 *    - If it oscillates or overshoots, lower kP slightly
 *
 * 2. Add a tiny kD (0.0005–0.002)
 *    - This helps dampen oscillation
 *
 * 3. kI is usually NOT needed for Limelight alignment
 *
 * 4. Tune while standing 8–12 feet from a tag
 * 5. Tune again while driving forward to ensure stability
 */
public class AutoAlignToHubPID extends Command {

  private final Drive swerve;
  private final VisionPositionValues vision;

  // Driver translation inputs (so you can still drive while aligning)
  private final DoubleSupplier xSpeed;
  private final DoubleSupplier ySpeed;

  // PID controller for rotation
  private final PIDController pid = new PIDController(0.03, 0.0, 0.001);

  public AutoAlignToHubPID(
      Drive swerve,
      VisionPositionValues vision,
      DoubleSupplier xSpeed,
      DoubleSupplier ySpeed) {

    this.swerve = swerve;
    this.vision = vision;
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;

    pid.setSetpoint(0);      // We want tx = 0 (centered)
    pid.setTolerance(1.0);   // Acceptable alignment error

    addRequirements(swerve);
  }

  @Override
  public void execute() {

    double x = xSpeed.getAsDouble();   // Driver forward/back
    double y = ySpeed.getAsDouble();   // Driver strafe

    // If no target, drive normally with no rotation correction
    if (!vision.hasTarget()) {
      swerve.drive(x, y, 0, true);
      return;
    }

    // Heading error from LL3/LL4
    double error = vision.getHeadingError();

    // PID output → rotational speed
    double rotation = pid.calculate(error);

    // Drive with driver translation + auto rotation
    swerve.drive(x, y, rotation, true);
  }

  @Override
  public boolean isFinished() {
    // This command runs continuously while RB is held
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    // Stop rotation when RB is released
    swerve.drive(0, 0, 0, true);
  }
}
