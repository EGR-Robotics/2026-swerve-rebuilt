package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.AprilTagVisionIO.AprilTagVisionIOInputs;
import frc.robot.FieldConstants;
import frc.robot.PoseEstimator.TimestampedVisionUpdate;
import frc.robot.VisionHelpers.PoseEstimate;
import java.util.*;
import java.util.function.Consumer;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

/**
 * This subsystem fuses AprilTag pose estimates from multiple cameras.
 *
 * It does NOT talk to Limelight directly — that happens in the IO layer.
 * 
 * Responsibilities:
 *  - Pull pose estimates from each camera
 *  - Filter out bad estimates
 *  - Compute measurement noise (std dev)
 *  - Send updates to your robot pose estimator
 *  - Log everything to AdvantageKit
 */
@Getter
@Setter
public class AprilTagVision extends SubsystemBase {

  /** How long a tag is considered "recently seen" for logging */
  private static final double TAG_LOG_WINDOW = 0.1;

  /** Field border margin to reject bad poses */
  private static final double FIELD_MARGIN = 0.5;

  /** Z height margin to reject bad poses */
  private static final double Z_MARGIN = 0.75;

  /** Noise scaling constants */
  private static final double XY_STD_COEFF = 0.01;
  private static final double THETA_STD_COEFF = 0.01;

  /** Path for AdvantageKit logging */
  private static final String LOG_PATH = "AprilTagVision/Camera";

  /** Whether to send updates to the pose estimator */
  private boolean enableVisionUpdates = true;

  /** Consumer that receives timestamped vision updates */
  private Consumer<List<TimestampedVisionUpdate>> visionConsumer = x -> {};

  /** One IO instance per camera */
  private final AprilTagVisionIO[] io;

  /** Inputs for each camera */
  private final AprilTagVisionIOInputs[] inputs;

  /** Last time each tag was seen (for logging) */
  private final Map<Integer, Double> lastTagSeen = new HashMap<>();

  /** Last known robot pose from vision */
  private Pose2d robotPose = new Pose2d();

  public AprilTagVision(AprilTagVisionIO... io) {
    this.io = io;
    this.inputs = new AprilTagVisionIOInputs[io.length];

    for (int i = 0; i < io.length; i++) {
      inputs[i] = new AprilTagVisionIOInputs();
    }

    // Initialize tag detection timestamps
    FieldConstants.aprilTags.getTags()
        .forEach(tag -> lastTagSeen.put(tag.ID, 0.0));
  }

  /** Allows the pose estimator to subscribe to vision updates */
  public void setDataInterfaces(Consumer<List<TimestampedVisionUpdate>> consumer) {
    this.visionConsumer = consumer;
  }

  @Override
  public void periodic() {

    // 1. Pull inputs from each camera
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs(LOG_PATH + i, inputs[i]);
    }

    // 2. Process pose estimates
    List<TimestampedVisionUpdate> updates = processPoseEstimates();

    // 3. Send to pose estimator
    if (enableVisionUpdates) {
      visionConsumer.accept(updates);
    }
  }

  /** Converts raw PoseEstimates into TimestampedVisionUpdates */
  private List<TimestampedVisionUpdate> processPoseEstimates() {
    List<TimestampedVisionUpdate> updates = new ArrayList<>();

    for (int cam = 0; cam < io.length; cam++) {
      for (PoseEstimate est : inputs[cam].poseEstimates) {

        if (shouldReject(est)) continue;

        Pose3d pose3d = est.pose();
        robotPose = pose3d.toPose2d();

        // Compute noise based on distance + number of tags
        int tagCount = est.tagIDs().length;
        double xyStd = XY_STD_COEFF * Math.pow(est.averageTagDistance(), 2) / tagCount;
        double thetaStd = THETA_STD_COEFF * Math.pow(est.averageTagDistance(), 2) / tagCount;

        updates.add(
            new TimestampedVisionUpdate(
                est.timestampSeconds(),
                robotPose,
                VecBuilder.fill(xyStd, xyStd, thetaStd)));

        logEstimate(cam, est);
      }
    }

    return updates;
  }

  /** Rejects bad estimates (no tags, null pose, outside field) */
  private boolean shouldReject(PoseEstimate est) {
    if (est.tagIDs().length < 1) return true;
    if (est.pose() == null) return true;

    Pose3d p = est.pose();

    return p.getX() < -FIELD_MARGIN
        || p.getX() > FieldConstants.fieldLength + FIELD_MARGIN
        || p.getY() < -FIELD_MARGIN
        || p.getY() > FieldConstants.fieldWidth + FIELD_MARGIN
        || p.getZ() < -Z_MARGIN
        || p.getZ() > Z_MARGIN;
  }

  /** Logs pose + tag info to AdvantageKit */
  private void logEstimate(int cam, PoseEstimate est) {

    Pose3d pose = est.pose();
    Logger.recordOutput(LOG_PATH + cam + "/RobotPose2d", pose.toPose2d());
    Logger.recordOutput(LOG_PATH + cam + "/RobotPose3d", pose);

    // Update last-seen timestamps
    for (int id : est.tagIDs()) {
      lastTagSeen.put(id, Timer.getFPGATimestamp());
    }

    // Log recently seen tags
    List<Pose3d> recent = new ArrayList<>();
    for (var entry : lastTagSeen.entrySet()) {
      if (Timer.getFPGATimestamp() - entry.getValue() < TAG_LOG_WINDOW) {
        FieldConstants.aprilTags.getTagPose(entry.getKey())
            .ifPresent(recent::add);
      }
    }

    Logger.recordOutput("AprilTagVision/RecentTags",
        recent.toArray(new Pose3d[0]));
  }
}
