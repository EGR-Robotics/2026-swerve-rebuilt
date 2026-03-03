package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.LimelightHelpers;

public class Vision {

    private static final String FRONT_LL = "limelight-front";
    private static final String BACK_LL  = "limelight-back";

    private static final int[] BLUE_HUB_TAGS = {26, 25, 27, 24, 18, 21};
    private static final int[] RED_HUB_TAGS  = {10, 9, 11, 8, 2, 5};

    private static final int[] BLUE_CLIMB_TAGS = {31, 32};
    private static final int[] RED_CLIMB_TAGS  = {15, 16};

    private static final double HUB_TO_CLIMB_DISTANCE = 3.00; //Todo: <-- UPDATE THIS

    public Vision() {
        LimelightHelpers.setPipelineIndex("limelight-front", 0);
    }

    /** Returns yaw error to hub or fallback climb tag direction */
    public double getYawToHub() {

        int[] hubTags = getHubTagList();
        int[] climbTags = getClimbTagList();

        // 1. Try front LL for hub tags
        double yaw = getYawFromCamera(FRONT_LL, hubTags);
        if (!Double.isNaN(yaw)) {
            return yaw;
        }

        // 2. Try back LL for climb tag fallback
        yaw = getYawFromCamera(BACK_LL, climbTags);
        if (!Double.isNaN(yaw)) {
            return convertClimbYawToHubYaw(yaw);
        }

        return Double.NaN;
    }

    /** Returns distance to hub from front camera only — no fallback to avoid inaccuracy */
    public double getDistanceToHub() {
        int[] hubTags = getHubTagList();
        return getDistanceFromCamera(FRONT_LL, hubTags);
    }

    private int[] getHubTagList() {
        return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
            ? BLUE_HUB_TAGS
            : RED_HUB_TAGS;
    }

    private int[] getClimbTagList() {
        return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
            ? BLUE_CLIMB_TAGS
            : RED_CLIMB_TAGS;
    }

    private double getYawFromCamera(String cam, int[] priorityList) {
        if (!LimelightHelpers.getTV(cam)) return Double.NaN;

        int detectedID = (int) LimelightHelpers.getFiducialID(cam);

        for (int id : priorityList) {
            if (id == detectedID) {
                return LimelightHelpers.getTX(cam);
            }
        }
        return Double.NaN;
    }

    private double getDistanceFromCamera(String cam, int[] priorityList) {
        if (!LimelightHelpers.getTV(cam)) return Double.NaN;

        int detectedID = (int) LimelightHelpers.getFiducialID(cam);

        for (int id : priorityList) {
            if (id == detectedID) {
                double[] pose = LimelightHelpers.getTargetPose_TargetSpace(cam);
                double x = pose[0];
                double z = pose[2];
                return Math.sqrt(x*x + z*z);
            }
        }
        return Double.NaN;
    }

    /** Negated: back camera faces away from hub, so yaw direction is inverted */
    private double convertClimbYawToHubYaw(double climbYaw) {
        return -climbYaw * 0.5; // tunable
    }
}