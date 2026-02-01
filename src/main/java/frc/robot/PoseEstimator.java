package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;

import java.util.*;

/**
 * PoseEstimator is a lightweight pose fusion engine originally based on
 * Mechanical Advantage (FRC 6328)'s 2024 implementation.
 *
 * It fuses:
 *   • Drive odometry (Twist2d)
 *   • AprilTag vision updates (Pose2d + std devs)
 *
 * This class is intentionally simple and does NOT depend on WPILib’s
 * SwerveDrivePoseEstimator. It is extremely stable and easy to debug.
 *
 * Your AprilTagVision subsystem will call:
 *   poseEstimator.addVisionData(...)
 *
 * Your Swerve subsystem will call:
 *   poseEstimator.addDriveData(...)
 */
public class PoseEstimator {

    /** How long to keep history for retroactive vision fusion */
    private static final double HISTORY_LENGTH_SECS = 0.3;

    /** The “base” pose after removing old history */
    private Pose2d basePose = new Pose2d();

    /** The most recent fused pose */
    private Pose2d latestPose = new Pose2d();

    /**
     * A timestamp → PoseUpdate map.
     * Each entry contains:
     *   • A drive twist
     *   • Zero or more vision updates
     */
    private final NavigableMap<Double, PoseUpdate> updates = new TreeMap<>();

    /**
     * Process noise matrix (q) for drive odometry.
     * q = stateStdDevs²
     */
    private final Matrix<N3, N1> q = new Matrix<>(Nat.N3(), Nat.N1());

    /**
     * @param stateStdDevs Standard deviations for drive odometry noise.
     *                     Example: VecBuilder.fill(0.1, 0.1, 0.1)
     */
    public PoseEstimator(Matrix<N3, N1> stateStdDevs) {
        for (int i = 0; i < 3; i++) {
            q.set(i, 0, stateStdDevs.get(i, 0) * stateStdDevs.get(i, 0));
        }
    }

    /** Returns the latest fused robot pose. */
    public Pose2d getLatestPose() {
        return latestPose;
    }

    /** Resets the estimator to a known pose. */
    public void resetPose(Pose2d pose) {
        basePose = pose;
        updates.clear();
        update();
    }

    /**
     * Adds a new drive twist at a given timestamp.
     * Called by your Swerve subsystem every loop.
     */
    public void addDriveData(double timestamp, Twist2d twist) {
        updates.put(timestamp, new PoseUpdate(twist, new ArrayList<>()));
        update();
    }

    /**
     * Adds a list of timestamped vision updates.
     * Called by AprilTagVision.java once per loop.
     */
    public void addVisionData(List<TimestampedVisionUpdate> visionData) {

        for (var update : visionData) {
            double ts = update.timestamp();
            VisionUpdate visionUpdate = new VisionUpdate(update.pose(), update.stdDevs());

            if (updates.containsKey(ts)) {
                // Add to existing update
                var list = updates.get(ts).visionUpdates();
                list.add(visionUpdate);
                list.sort(VisionUpdate.compareDescStdDev);

            } else {
                // Insert between two drive updates
                var prev = updates.floorEntry(ts);
                var next = updates.ceilingEntry(ts);

                if (prev == null || next == null) {
                    // Vision timestamp outside history window
                    continue;
                }

                // Split the twist proportionally
                double totalDt = next.getKey() - prev.getKey();
                double alpha = (ts - prev.getKey()) / totalDt;

                Twist2d fullTwist = next.getValue().twist();
                Twist2d twist0 = multiplyTwist(fullTwist, alpha);
                Twist2d twist1 = multiplyTwist(fullTwist, 1 - alpha);

                // Insert new updates
                ArrayList<VisionUpdate> newList = new ArrayList<>();
                newList.add(visionUpdate);

                updates.put(ts, new PoseUpdate(twist0, newList));
                updates.put(next.getKey(), new PoseUpdate(twist1, next.getValue().visionUpdates()));
            }
        }

        update();
    }

    /** Recomputes the latest pose by applying all updates in order. */
    private void update() {

        // Remove old history and update base pose
        while (updates.size() > 1 &&
                updates.firstKey() < Timer.getFPGATimestamp() - HISTORY_LENGTH_SECS) {

            var entry = updates.pollFirstEntry();
            basePose = entry.getValue().apply(basePose, q);
        }

        // Recompute latest pose
        latestPose = basePose;
        for (var entry : updates.entrySet()) {
            latestPose = entry.getValue().apply(latestPose, q);
        }
    }

    /** Multiply a twist by a scalar (used for splitting drive updates). */
    private static Twist2d multiplyTwist(Twist2d twist, double scalar) {
        return new Twist2d(
                twist.dx * scalar,
                twist.dy * scalar,
                twist.dtheta * scalar
        );
    }

    /** A single update: drive twist + list of vision updates. */
    private static record PoseUpdate(Twist2d twist, ArrayList<VisionUpdate> visionUpdates) {

        public Pose2d apply(Pose2d lastPose, Matrix<N3, N1> q) {

            // Apply drive twist first
            Pose2d pose = lastPose.exp(twist);

            // Apply each vision update
            for (var vision : visionUpdates) {

                // Compute Kalman gain
                Matrix<N3, N3> K = new Matrix<>(Nat.N3(), Nat.N3());
                double[] r = new double[3];

                for (int i = 0; i < 3; i++) {
                    r[i] = vision.stdDevs().get(i, 0) * vision.stdDevs().get(i, 0);
                }

                for (int i = 0; i < 3; i++) {
                    double qi = q.get(i, 0);
                    K.set(i, i, qi == 0 ? 0 : qi / (qi + Math.sqrt(qi * r[i])));
                }

                // Compute twist from current pose → vision pose
                Twist2d visionTwist = pose.log(vision.pose());

                // Apply Kalman gain
                var twistMatrix = K.times(VecBuilder.fill(
                        visionTwist.dx,
                        visionTwist.dy,
                        visionTwist.dtheta));

                pose = pose.exp(new Twist2d(
                        twistMatrix.get(0, 0),
                        twistMatrix.get(1, 0),
                        twistMatrix.get(2, 0)));
            }

            return pose;
        }
    }

    /** A single vision measurement with standard deviations. */
    public static record VisionUpdate(Pose2d pose, Matrix<N3, N1> stdDevs) {
        public static final Comparator<VisionUpdate> compareDescStdDev =
                (a, b) -> -Double.compare(
                        a.stdDevs().get(0, 0) + a.stdDevs().get(1, 0),
                        b.stdDevs().get(0, 0) + b.stdDevs().get(1, 0));
    }

    /** A timestamped vision update (used by AprilTagVision). */
    public record TimestampedVisionUpdate(double timestamp, Pose2d pose, Matrix<N3, N1> stdDevs) {}
}
