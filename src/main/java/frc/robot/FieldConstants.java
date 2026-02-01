package frc.robot.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import java.io.IOException;

/**
 * FieldConstants for FRC 2026.
 *
 * This file defines:
 *  - Field dimensions
 *  - Hub locations
 *  - AprilTag layout (loaded from WPILib)
 *  - Useful reference points for auto-aligning
 *
 * NOTE:
 *  WPILib has not yet published the official 2026 field layout.
 *  When it becomes available, replace:
 *
 *      AprilTagFields.k2026Field
 *
 *  with the correct enum value.
 */
public class FieldConstants {

  /** -----------------------------
   *  FIELD DIMENSIONS (PLACEHOLDERS)
   *  Replace these with official 2026 values when published.
   *  ----------------------------- */
  public static double fieldLength = Units.inchesToMeters(651.223); // placeholder
  public static double fieldWidth  = Units.inchesToMeters(323.277); // placeholder

  /** -----------------------------
   *  HUB LOCATIONS (YOU WILL UPDATE THESE)
   *  These are used for auto-aligning to the hub.
   *  ----------------------------- */

  // Replace with official hub center coordinates (Blue Alliance)
  public static Translation2d blueHubCenter =
      new Translation2d(
          Units.inchesToMeters(0),     // placeholder X
          Units.inchesToMeters(0));    // placeholder Y

  // Replace with official hub center coordinates (Red Alliance)
  public static Translation2d redHubCenter =
      new Translation2d(
          Units.inchesToMeters(0),     // placeholder X
          Units.inchesToMeters(0));    // placeholder Y

  /** -----------------------------
   *  HUB APRILTAG IDs (YOU WILL UPDATE THESE)
   *  ----------------------------- */
  public static final int[] blueHubTagIDs = {
      /* e.g. 1, 2, 3 */ 
  };

  public static final int[] redHubTagIDs = {
      /* e.g. 4, 5, 6 */
  };

  /** -----------------------------
   *  APRILTAG LAYOUT
   *  ----------------------------- */
  public static AprilTagFieldLayout aprilTags;

  static {
    try {
      // Replace k2026Field with the correct enum once WPILib publishes it
      aprilTags = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
      // ^ TEMPORARY — replace with:
      // aprilTags = AprilTagFields.k2026Field.loadAprilTagLayoutField();

    } catch (IOException e) {
      throw new RuntimeException("Failed to load 2026 AprilTag layout", e);
    }
  }

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
