package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * FieldConstants for FRC 2026.
 *
 * This file defines:
 *  - Field dimensions (placeholder until WPILib publishes official values)
 *  - Hub center positions (placeholder)
 *  - Hub AprilTag IDs (placeholder)
 *  - AprilTag layout loader for 2026
 *  - Helper for alliance‑aware hub lookup
 *
 * Once WPILib publishes the 2026 field, update:
 *   AprilTagFields.k2026Field
 *   hub center coordinates
 *   hub tag IDs
 */
public class FieldConstants {

    /** -----------------------------
     *  FIELD DIMENSIONS (PLACEHOLDERS)
     *  Replace with official 2026 values when published.
     *  ----------------------------- */
    public static final double fieldLength = Units.inchesToMeters(651.22); 
    public static final double fieldWidth  = Units.inchesToMeters(317.69); 

    /** -----------------------------
     *  HUB LOCATIONS (PLACEHOLDERS)
     *  Used for auto-aligning to the hub.
     *  Replace with official 2026 coordinates.
     *  ----------------------------- */

    // Blue alliance hub center
    public static Translation2d blueHubCenter =
            new Translation2d(
                    Units.inchesToMeters(182.11),  // X 
                    Units.inchesToMeters(158.84));  // Y

    // Red alliance hub center
    public static Translation2d redHubCenter =
            new Translation2d(
                    Units.inchesToMeters(182.11),   // X
                    Units.inchesToMeters(158.84));  // Y
    
    public static final double hubCenterZ = Units.inchesToMeters(72.0);

    /** -----------------------------
     *  HUB APRILTAG IDs (PLACEHOLDERS)
     *  Replace with official tag IDs once published.
     *  ----------------------------- */
    public static final int[] blueHubTagIDs = {
            18, 27, 26, 25, 21, 24, 20, 19
    };

    public static final int[] redHubTagIDs = {
            8, 5, 9, 10, 11, 2, 3, 4
    };

    /** -----------------------------
     *  APRILTAG LAYOUT (2026)
     *  No try/catch needed — WPILib 2026 loader does not throw IOException.
     *  ----------------------------- */
    public static final AprilTagFieldLayout aprilTags =
            AprilTagFields.k2026RebuiltWelded.loadAprilTagLayoutField();
            // Change to AndyMark instead of Welded if/when we make it to worlds.
    /** -----------------------------
     *  AUTO-ALIGN HELPERS
     *  ----------------------------- */

    /**
     * Returns the hub center for the current alliance.
     * Used by auto-align commands to compute desired heading.
     */
    public static Translation2d getHubCenter(boolean isRedAlliance) {
        return isRedAlliance ? redHubCenter : blueHubCenter;
    }
}
