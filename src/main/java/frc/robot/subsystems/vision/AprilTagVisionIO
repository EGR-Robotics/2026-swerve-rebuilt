package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.VisionHelpers;
import frc.robot.util.VisionHelpers.PoseEstimate;
import java.util.ArrayList;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/**
 * This interface defines the "IO layer" for any AprilTag camera.
 * 
 * Each camera type (Limelight, PhotonVision, simulation, etc.)
 * implements updateInputs() to populate poseEstimates.
 *
 * AprilTagVision.java consumes these estimates and fuses them
 * into the robot pose estimator.
 */
public interface AprilTagVisionIO {

  @Setter
  @Getter
  class AprilTagVisionIOInputs implements LoggableInputs {

    /** 
     * A list of pose estimates from the camera.
     * Each PoseEstimate contains:
     *  - Pose3d robot pose
     *  - Timestamp
     *  - Average tag distance
     *  - Array of tag IDs used
     */
    public ArrayList<PoseEstimate> poseEstimates = new ArrayList<>();

    @Override
    public void toLog(LogTable table) {
      table.put("poseEstimates", poseEstimates.size());

      for (int i = 0; i < poseEstimates.size(); i++) {
        PoseEstimate est = poseEstimates.get(i);

        table.put("estimatedPose/" + i,
            VisionHelpers.getPose3dToArray(est.pose()));

        table.put("timestamp/" + i, est.timestampSeconds());
        table.put("tagIDs/" + i, est.tagIDs());
        table.put("avgTagDistance/" + i, est.averageTagDistance());
      }

      table.put("valid", !poseEstimates.isEmpty());
    }

    @Override
    public void fromLog(LogTable table) {
      poseEstimates.clear();

      int count = table.get("poseEstimates", 0);

      for (int i = 0; i < count; i++) {
        Pose3d pose =
            LimelightHelpers.toPose3D(table.get("estimatedPose/" + i, new double[] {}));

        double timestamp = table.get("timestamp/" + i, 0.0);
        double avgDist = table.get("avgTagDistance/" + i, 0.0);
        int[] ids = table.get("tagIDs/" + i, new int[] {});

        poseEstimates.add(new PoseEstimate(pose, timestamp, avgDist, ids));
      }
    }
  }

  /** Implemented by each camera type */
  default void updateInputs(AprilTagVisionIOInputs inputs) {}
}
