package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Simple subsystem that exposes tx/ty/ta from two Limelights.
 * Used for alignment and debugging.
 */
public class VisionPositionValues extends SubsystemBase {

  // LL3 (front)
  private final NetworkTable ll3 = NetworkTableInstance.getDefault().getTable("limelight-one");
  private final NetworkTableEntry tx3 = ll3.getEntry("tx");
  private final NetworkTableEntry ty3 = ll3.getEntry("ty");
  private final NetworkTableEntry ta3 = ll3.getEntry("ta");

  // LL4 (high)
  private final NetworkTable ll4 = NetworkTableInstance.getDefault().getTable("limelight-two");
  private final NetworkTableEntry tx4 = ll4.getEntry("tx");
  private final NetworkTableEntry ty4 = ll4.getEntry("ty");
  private final NetworkTableEntry ta4 = ll4.getEntry("ta");

  // Cached values
  private double txLL3, tyLL3, taLL3;
  private double txLL4, tyLL4, taLL4;

  @Override
  public void periodic() {
    txLL3 = tx3.getDouble(0.0);
    tyLL3 = ty3.getDouble(0.0);
    taLL3 = ta3.getDouble(0.0);

    txLL4 = tx4.getDouble(0.0);
    tyLL4 = ty4.getDouble(0.0);
    taLL4 = ta4.getDouble(0.0);

    SmartDashboard.putNumber("LL3/tx", txLL3);
    SmartDashboard.putNumber("LL3/ty", tyLL3);
    SmartDashboard.putNumber("LL3/ta", taLL3);

    SmartDashboard.putNumber("LL4/tx", txLL4);
    SmartDashboard.putNumber("LL4/ty", tyLL4);
    SmartDashboard.putNumber("LL4/ta", taLL4);
  }

  /** Prefer LL4 if it sees a tag, otherwise LL3. */
  public double getHeadingError() {
    if (taLL4 > 0.1) return txLL4;
    return txLL3;
  }

  public boolean hasTarget() {
    return taLL4 > 0.1 || taLL3 > 0.1;
  }
}
