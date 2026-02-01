package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.LimelightHelpers;
import frc.robot.VisionHelpers.PoseEstimate;

import java.util.ArrayList;
import java.util.Optional;


/**
 * Limelight IO layer for 2026 API (v1.14).
 * Uses getLatestResults() instead of parseJsonDump().
 */
public class AprilTagVisionIOLimelight implements AprilTagVisionIO {

    private final String llName;

    private final NetworkTableEntry txEntry;
    private final NetworkTableEntry tyEntry;
    private final NetworkTableEntry taEntry;

    private double tx = 0;
    private double ty = 0;
    private double ta = 0;

    public AprilTagVisionIOLimelight(String tableName, String llName, int pipeline) {
        this.llName = llName;

        NetworkTable table = NetworkTableInstance.getDefault().getTable(tableName);

        txEntry = table.getEntry("tx");
        tyEntry = table.getEntry("ty");
        taEntry = table.getEntry("ta");

        LimelightHelpers.setPipelineIndex(llName, pipeline);
    }

    @Override
    public void updateInputs(AprilTagVisionIOInputs inputs) {

        // Basic target info
        tx = txEntry.getDouble(0);
        ty = tyEntry.getDouble(0);
        ta = taEntry.getDouble(0);

        // Get full results from Limelight
        LimelightHelpers.LimelightResults results =
                LimelightHelpers.getLatestResults(llName);

        ArrayList<PoseEstimate> estimates = new ArrayList<>();

        if (results == null || results.targets_Fiducials.length == 0) {
            inputs.poseEstimates = estimates;
            return;
        }

        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) {
            inputs.poseEstimates = estimates;
            return;
        }

        // Alliance-corrected pose
        Pose3d pose =
                alliance.get() == Alliance.Red
                        ? results.getBotPose3d_wpiRed()
                        : results.getBotPose3d_wpiBlue();

        // Timestamp correction
        double timestamp =
                (results.timestamp_us / 1e6) -
                ((results.latency_capture + results.latency_pipeline) / 1000.0);

        // Average tag distance
        double avgDist = results.botpose_avgdist;

        // Tag IDs
        int[] ids = new int[results.targets_Fiducials.length];
        for (int i = 0; i < ids.length; i++) {
            ids[i] = (int) results.targets_Fiducials[i].fiducialID;
        }

        estimates.add(new PoseEstimate(pose, timestamp, avgDist, ids));
        inputs.poseEstimates = estimates;
    }

    public double getTx() { return tx; }
    public double getTy() { return ty; }
    public double getTa() { return ta; }
    public boolean hasTarget() { return ta > 0.1; }

}
