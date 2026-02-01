package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import java.util.Arrays;
import java.util.Objects;

/**
 * VisionHelpers contains small utility structures and conversion helpers
 * used by the AprilTag vision pipeline.
 *
 * This class is intentionally lightweight — it simply defines:
 *   • A PoseEstimate record (pose + timestamp + tag info)
 *   • A helper to convert Pose3d → double[6] for logging
 *
 * AprilTagVisionIO implementations create PoseEstimate objects,
 * and AprilTagVision.java consumes them for pose fusion.
 */
public class VisionHelpers {

  /**
   * Represents a single AprilTag-based pose estimate from a camera.
   *
   * Fields:
   *   pose               → The estimated robot Pose3d
   *   timestampSeconds   → Timestamp (corrected for latency)
   *   averageTagDistance → Average distance to all detected tags
   *   tagIDs             → IDs of the tags used in this estimate
   *
   * This record is used by:
   *   • AprilTagVisionIOLimelight
   *   • AprilTagVisionIOPhotonVisionSIM
   *   • AprilTagVision (fusion subsystem)
   */
  public record PoseEstimate(
      Pose3d pose,
      double timestampSeconds,
      double averageTagDistance,
      int[] tagIDs
  ) {

    /** 
     * Custom equality check so AdvantageKit logging and pose fusion
     * can reliably compare PoseEstimate objects.
     */
    @Override
    public boolean equals(Object obj) {
      if (this == obj) return true;
      if (obj == null || getClass() != obj.getClass()) return false;

      PoseEstimate other = (PoseEstimate) obj;

      return Arrays.equals(tagIDs, other.tagIDs)
          && Objects.equals(pose, other.pose)
          && Double.compare(timestampSeconds, other.timestampSeconds) == 0
          && Double.compare(averageTagDistance, other.averageTagDistance) == 0;
    }

    /**
     * Hash code based on pose, timestamp, distance, and tag IDs.
     * Used by logging and any hash-based collections.
     */
    @Override
    public int hashCode() {
      return Objects.hash(
          Arrays.hashCode(getPose3dToArray(pose)),
          timestampSeconds,
          averageTagDistance,
          Arrays.hashCode(tagIDs));
    }

    /**
     * Human-readable string for debugging.
     */
    @Override
    public String toString() {
      return "PoseEstimate{"
          + "pose=" + pose
          + ", timestampSeconds=" + timestampSeconds
          + ", averageTagDistance=" + averageTagDistance
          + ", tagIDs=" + Arrays.toString(tagIDs)
          + '}';
    }
  }

  /**
   * Converts a Pose3d into a 6-element array for logging and AdvantageKit.
   *
   * Format:
   *   [x, y, z, rollDeg, pitchDeg, yawDeg]
   *
   * This matches the format used by:
   *   • AprilTagVisionIOInputs.toLog()
   *   • AdvantageKit pose logging
   */
  public static double[] getPose3dToArray(Pose3d pose) {
    return new double[] {
        pose.getX(),
        pose.getY(),
        pose.getZ(),
        Units.radiansToDegrees(pose.getRotation().getX()),
        Units.radiansToDegrees(pose.getRotation().getY()),
        Units.radiansToDegrees(pose.getRotation().getZ())
    };
  }
}
