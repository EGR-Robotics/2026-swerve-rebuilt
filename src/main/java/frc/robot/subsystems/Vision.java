// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import frc.robot.LimelightHelpers;

// public class Vision {

//     private static final String FRONT_LL = "limelight-front";
//     private static final String BACK_LL  = "limelight-back";

//     private static final int[] BLUE_HUB_TAGS = {26};
//     private static final int[] RED_HUB_TAGS  = {10};

//     private static final int[] BLUE_CLIMB_TAGS = {31, 32};
//     private static final int[] RED_CLIMB_TAGS  = {15, 16};

//     private static final double HUB_TO_CLIMB_DISTANCE = 3.00;

//     public Vision() {
//         // Constructor MUST stay empty to avoid delaying auto
//     }

//     /** Call this ONCE in robotInit() */
//     public void initialize() {
//         System.out.println("[Vision] Initializing Limelights...");
//         LimelightHelpers.setPipelineIndex(FRONT_LL, 0);
//         LimelightHelpers.setPipelineIndex(BACK_LL, 0);
//         System.out.println("[Vision] Initialization complete.");
//     }

//     /** Returns yaw error to hub or fallback climb tag direction */
//     public double getYawToHub() {

//         int[] hubTags = getHubTagList();
//         int[] climbTags = getClimbTagList();

//         double yaw = getYawFromCamera(FRONT_LL, hubTags) * 1.1;
//         if (!Double.isNaN(yaw)) {
//             return yaw;
//         }

//         yaw = getYawFromCamera(BACK_LL, climbTags) * 1.1;
//         if (!Double.isNaN(yaw)) {
//             return convertClimbYawToHubYaw(yaw);
//         }

//         return Double.NaN;
//     }

//     public double getDistanceToHub() {
//         int[] hubTags = getHubTagList();
//         return getDistanceFromCamera(FRONT_LL, hubTags);
//     }

//     private int[] getHubTagList() {
//         return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
//             ? BLUE_HUB_TAGS
//             : RED_HUB_TAGS;
//     }

//     private int[] getClimbTagList() {
//         return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
//             ? BLUE_CLIMB_TAGS
//             : RED_CLIMB_TAGS;
//     }

//     private double getYawFromCamera(String cam, int[] priorityList) {
//         if (!LimelightHelpers.getTV(cam)) return Double.NaN;

//         int detectedID = (int) LimelightHelpers.getFiducialID(cam);

//         for (int id : priorityList) {
//             if (id == detectedID) {
//                 return LimelightHelpers.getTX(cam);
//             }
//         }
//         return Double.NaN;
//     }

//     private double getDistanceFromCamera(String cam, int[] priorityList) {
//         if (!LimelightHelpers.getTV(cam)) return Double.NaN;

//         int detectedID = (int) LimelightHelpers.getFiducialID(cam);

//         for (int id : priorityList) {
//             if (id == detectedID) {
//                 double[] pose = LimelightHelpers.getTargetPose_TargetSpace(cam);
//                 double x = pose[0];
//                 double z = pose[2];
//                 return Math.sqrt(x*x + z*z);
//             }
//         }
//         return Double.NaN;
//     }

//     private double convertClimbYawToHubYaw(double climbYaw) {
//         return -climbYaw * 0.5;
//     }
// }
