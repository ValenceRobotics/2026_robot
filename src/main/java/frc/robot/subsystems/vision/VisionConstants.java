// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

  // Camera names, must match names configured on coprocessor
  public static String cameraFName = "camera_f";
  public static String cameraLName = "camera_l";
  public static String cameraRName = "camera_r";

  /* two front facing cameras */
  public static Transform3d robotToCameraF =
      new Transform3d(
          0.343, 0.153988, 0.212343, new Rotation3d(0.0, Units.degreesToRadians(-20.0), 0));
  public static Transform3d robotToCameraL =
      new Transform3d(
          0.000,
          -0.343,
          0.196850,
          new Rotation3d(
              0, Units.degreesToRadians(-15), Units.degreesToRadians(-55))); // left of shooter
  public static Transform3d robotToCameraR =
      new Transform3d(
          0.000,
          0.343,
          0.196850,
          new Rotation3d(
              0, Units.degreesToRadians(-15), Units.degreesToRadians(55))); // right of shooter

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        1.0 // Camera 1
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available
}
