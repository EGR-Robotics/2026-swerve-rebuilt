/**
 * LimelightHelpers - a helper class for interfacing with Limelight NetworkTables data.
 * This file is maintained by Limelight Vision and is safe to copy directly into your robot code.
 */

package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightHelpers {

    public static class PoseEstimate {
        public double timestampSeconds;
        public double[] pose;
        public double[] avgTagDist;
        public double[] avgTagArea;
        public double latency;
        public int tagCount;
        public int[] tagIDs;
    }

    private static NetworkTable getTable(String tableName) {
        return NetworkTableInstance.getDefault().getTable(tableName);
    }

    public static boolean getTV(String tableName) {
        return getTable(tableName).getEntry("tv").getDouble(0) == 1;
    }

    public static double getTX(String tableName) {
        return getTable(tableName).getEntry("tx").getDouble(0);
    }

    public static double getTY(String tableName) {
        return getTable(tableName).getEntry("ty").getDouble(0);
    }

    public static double getTA(String tableName) {
        return getTable(tableName).getEntry("ta").getDouble(0);
    }

    public static double getFiducialID(String tableName) {
        return getTable(tableName).getEntry("tid").getDouble(-1);
    }

    public static double[] getBotPose(String tableName) {
        return getTable(tableName).getEntry("botpose").getDoubleArray(new double[6]);
    }

    public static double[] getBotPose_wpiBlue(String tableName) {
        return getTable(tableName).getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
    }

    public static double[] getBotPose_wpiRed(String tableName) {
        return getTable(tableName).getEntry("botpose_wpired").getDoubleArray(new double[6]);
    }

    public static double getLatency(String tableName) {
        return getTable(tableName).getEntry("tl").getDouble(0);
    }

    public static PoseEstimate getBotPoseEstimate(String tableName) {
        PoseEstimate est = new PoseEstimate();

        double[] pose = getBotPose(tableName);
        est.pose = pose;

        double latency = getLatency(tableName);
        est.latency = latency;

        double timestamp = edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - (latency / 1000.0);
        est.timestampSeconds = timestamp;

        double[] tid = getTable(tableName).getEntry("tid").getDoubleArray(new double[0]);
        est.tagIDs = new int[tid.length];
        for (int i = 0; i < tid.length; i++) {
            est.tagIDs[i] = (int) tid[i];
        }

        est.tagCount = est.tagIDs.length;

        return est;
    }

    public static void setLEDMode(String tableName, int mode) {
        getTable(tableName).getEntry("ledMode").setNumber(mode);
    }

    public static void setPipelineIndex(String tableName, int index) {
        getTable(tableName).getEntry("pipeline").setNumber(index);
    }

    public static void setCameraMode(String tableName, int mode) {
        getTable(tableName).getEntry("camMode").setNumber(mode);
    }

    public static void setStreamMode(String tableName, int mode) {
        getTable(tableName).getEntry("stream").setNumber(mode);
    }

    public static void setSnapshotMode(String tableName, int mode) {
        getTable(tableName).getEntry("snapshot").setNumber(mode);
    }

    public static void takeSnapshot(String tableName, String name) {
        getTable(tableName).getEntry("snapshot").setString(name);
    }

    public static void setCrop(String tableName, double x0, double x1, double y0, double y1) {
        getTable(tableName).getEntry("crop").setDoubleArray(new double[]{x0, x1, y0, y1});
    }

    public static double[] getTargetPose_CameraSpace(String tableName) {
        return getTable(tableName).getEntry("targetpose_cameraspace").getDoubleArray(new double[6]);
    }

    public static double[] getTargetPose_RobotSpace(String tableName) {
        return getTable(tableName).getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
    }

    public static double[] getTargetPose_FieldSpace(String tableName) {
        return getTable(tableName).getEntry("targetpose_fieldspace").getDoubleArray(new double[6]);
    }

    public static double[] getTargetPose_TargetSpace(String tableName) {
        return getTable(tableName).getEntry("targetpose_targetspace").getDoubleArray(new double[6]);
    }

    public static double[] getCameraPose_TargetSpace(String tableName) {
        return getTable(tableName).getEntry("camerapose_targetspace").getDoubleArray(new double[6]);
    }

    public static double[] getCameraPose_RobotSpace(String tableName) {
        return getTable(tableName).getEntry("camerapose_robotspace").getDoubleArray(new double[6]);
    }

    public static double[] getCameraPose_FieldSpace(String tableName) {
        return getTable(tableName).getEntry("camerapose_fieldspace").getDoubleArray(new double[6]);
    }

    public static double[] getRobotPose_TargetSpace(String tableName) {
        return getTable(tableName).getEntry("robotpose_targetspace").getDoubleArray(new double[6]);
    }

    public static double[] getRobotPose_FieldSpace(String tableName) {
        return getTable(tableName).getEntry("robotpose_fieldspace").getDoubleArray(new double[6]);
    }

    public static double[] getRobotPose_CameraSpace(String tableName) {
        return getTable(tableName).getEntry("robotpose_cameraspace").getDoubleArray(new double[6]);
    }

    public static double[] getRobotPose_RobotSpace(String tableName) {
        return getTable(tableName).getEntry("robotpose_robotspace").getDoubleArray(new double[6]);
    }

    public static double getTargetArea(String tableName) {
        return getTable(tableName).getEntry("ta").getDouble(0);
    }

    public static double getTargetSkew(String tableName) {
        return getTable(tableName).getEntry("ts").getDouble(0);
    }

    public static double getTargetShort(String tableName) {
        return getTable(tableName).getEntry("tshort").getDouble(0);
    }

    public static double getTargetLong(String tableName) {
        return getTable(tableName).getEntry("tlong").getDouble(0);
    }

    public static double getTargetHorizontal(String tableName) {
        return getTable(tableName).getEntry("thor").getDouble(0);
    }

    public static double getTargetVertical(String tableName) {
        return getTable(tableName).getEntry("tvert").getDouble(0);
    }

    public static double[] getRawCorners(String tableName) {
        return getTable(tableName).getEntry("tcornx").getDoubleArray(new double[0]);
    }

    public static double[] getRawCornersY(String tableName) {
        return getTable(tableName).getEntry("tcorny").getDoubleArray(new double[0]);
    }

    public static double getLatencyPipeline(String tableName) {
        return getTable(tableName).getEntry("tl").getDouble(0);
    }

    public static double getLatencyCapture(String tableName) {
        return getTable(tableName).getEntry("cl").getDouble(0);
    }

    public static double getLatencyTotal(String tableName) {
        return getLatencyPipeline(tableName) + getLatencyCapture(tableName);
    }
}
