package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.VisionHelpers.PoseEstimate;
import java.util.ArrayList;
import java.util.Optional;

/**
 * This class is the hardware interface for a single Limelight camera.
 * It converts Limelight JSON output into PoseEstimate objects that the
 * AprilTagVision subsystem can fuse into the robot pose estimator.
 */
public class AprilTagVisionIOLimelight implements AprilTagVisionIO {

  // NetworkTables table name (e.g., "limelight-one")
  private final String tableName;

  // Name used by LimelightHelpers (usually same as tableName)
  private final String llName;

  // Subscriber for Limelight JSON pose data
  private final StringSubscriber jsonSubscriber;

  // Basic NT entries for tx/ty/ta (used for alignment)
  private final NetworkTableEntry txEntry;
  private final NetworkTableEntry tyEntry;
  private final NetworkTableEntry taEntry;

  // Cached values
  private double tx = 0.0;
  private double ty = 0.0;
  private double ta = 0.0;

  /**
   * @param tableName  NetworkTables table name (e.g., "limelight-one")
   * @param llName     LimelightHelpers name (same as tableName is fine)
   * @param pipeline   AprilTag pipeline index
   */
  public AprilTagVisionIOLimelight(String tableName, String llName, int pipeline) {
    this.tableName = tableName;
    this.llName = llName;

    NetworkTable table = NetworkTableInstance.getDefault().getTable(tableName);

    // Set AprilTag pipeline
    LimelightHelpers.setPipelineIndex(llName, pipeline);

    // Basic target info
    txEntry = table.getEntry("tx");
    tyEntry = table.getEntry("ty");
    taEntry = table.getEntry("ta");

    // Subscribe to JSON pose data
    jsonSubscriber =
        table.getStringTopic("json")
            .subscribe("", PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));
  }

  @Override
  public void updateInputs(AprilTagVisionIOInputs inputs) {

    // Read basic target info
    tx = txEntry.getDouble(0.0);
    ty = tyEntry.getDouble(0.0);
    ta = taEntry.getDouble(0.0);

    SmartDashboard.putNumber(tableName + "/tx", tx);
    SmartDashboard.putNumber(tableName + "/ty", ty);
    SmartDashboard.putNumber(tableName + "/ta", ta);

    // Read JSON queue
    TimestampedString[] queue = jsonSubscriber.readQueue();
    ArrayList<PoseEstimate> estimates = new ArrayList<>();

    for (TimestampedString packet : queue) {

      // Convert timestamp from microseconds → seconds
      double timestamp = packet.timestamp / 1e6;

      // Parse JSON
      LimelightHelpers.Results results =
          LimelightHelpers.parseJsonDump(packet.value).targetingResults;

      // Skip if no tags
      if (results.targets_Fiducials.length == 0) continue;

      Optional<Alliance> alliance = DriverStation.getAlliance();
      if (alliance.isEmpty()) continue;

      // Latency compensation
      double latencyMS = results.latency_capture + results.latency_pipeline;
      timestamp -= latencyMS / 1000.0;

      // Alliance‑corrected pose
      Pose3d pose =
          alliance.get() == Alliance.Red
              ? results.getBotPose3d_wpiRed()
              : results.getBotPose3d_wpiBlue();

      // Compute average tag distance
      double avgDist = 0.0;
      int[] ids = new int[results.targets_Fiducials.length];

      for (int i = 0; i < results.targets_Fiducials.length; i++) {
        ids[i] = (int) results.targets_Fiducials[i].fiducialID;
        avgDist +=
            results.targets_Fiducials[i]
                .getTargetPose_CameraSpace()
                .getTranslation()
                .getNorm();
      }

      avgDist /= ids.length;

      // Add estimate
      estimates.add(new PoseEstimate(pose, timestamp, avgDist, ids));
    }

    inputs.poseEstimates = estimates;
  }

  // Accessors for alignment
  public double getTx() { return tx; }
  public double getTy() { return ty; }
  public double getTa() { return ta; }
  public boolean hasTarget() { return ta > 0.1; }
}
