// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Constants.Mode;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // Camera names, must match names configured on coprocessor
  public static String cameraBackName = "back";
  public static String cameraLeftName = "left";
  public static String cameraRightName = "right";

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
  public static Transform3d robotToCameraBack =
      new Transform3d(
          Units.inchesToMeters(-13.4375),
          Units.inchesToMeters(1 + 1 / 8),
          Units.inchesToMeters(15 + (9 / 16)),
          new Rotation3d(0.0, 0, Units.degreesToRadians(180)));
  public static Transform3d robotToCameraLeft =
      new Transform3d(
          Units.inchesToMeters((-27.5 / 2) /* frame edge */ + (3 + (1 / 4)) /* in */),
          Units.inchesToMeters((27.5 / 2) /* frame edge */ - (2 + (9 / 16)) /* in */),
          Units.inchesToMeters(7 + (15 / 16) /* up */),
          new Rotation3d(
              0.0, Units.degreesToRadians(-23.05 /* pitch up */), Units.degreesToRadians(90)));
  public static Transform3d robotToCameraRight =
      new Transform3d(
          Units.inchesToMeters((-27 / 2) + (2 / 10)),
          Units.inchesToMeters((-27 / 2) + 8 + (1 / 2)),
          Units.inchesToMeters(6 + (7 / 16) + 1 + (5 / 8)),
          new Rotation3d(0.0, Units.degreesToRadians(-3.9), Units.degreesToRadians(-90)));

    static {
        if (Constants.currentMode == Mode.SIM) {
            Logger.recordOutput("Vision/Cameras/"+cameraBackName, robotToCameraBack);
            Logger.recordOutput("Vision/Cameras/"+cameraLeftName, robotToCameraLeft);
            Logger.recordOutput("Vision/Cameras/"+cameraRightName, robotToCameraRight);

        }
    }

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

// Back Camera
// 15 9/16 from the ground
// 1 1/8 to robot left
// 1/16 in from frame = -13.4375

// Right Camera
// 3 1/4 from back
// 2 9/16 from right frame
// 7 15/16 up
// 34.05 pitch
// 27.5 frame

// Left Camera
// 6 7/16 + 1 5/8 from the ground
// 8 3/4 from back big edge (27.5)
// 2/10 from edge, 27/2 - 2/10
