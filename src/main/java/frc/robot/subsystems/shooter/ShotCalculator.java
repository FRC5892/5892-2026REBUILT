// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotState;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants;
import frc.robot.util.FieldConstants.LinesHorizontal;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.Logger;

/** Thank you so much 6328! */
public class ShotCalculator {
  private static ShotCalculator instance;

  public static ShotCalculator getInstance() {
    if (instance == null) {
      instance = new ShotCalculator();
    }
    return instance;
  }

  public static final Transform2d robotToTurret =
      new Transform2d(
          new Translation2d(Units.inchesToMeters(-6), Units.inchesToMeters(-6)), new Rotation2d());

  private static final Translation2d rightTarget =
      AllianceFlipUtil.apply(new Translation2d(1.5, 1.5));

  private static final Translation2d leftTarget =
      rightTarget.plus(new Translation2d(0, (LinesHorizontal.center - rightTarget.getX()) * 2));

  private static final Translation2d centerTarget =
      new Translation2d(rightTarget.getX(), LinesHorizontal.center);

  private Rotation2d turretAngle;
  private Rotation2d hoodAngle = Rotation2d.kZero;

  public record ShotParameters(
      boolean isValid,
      Rotation2d turretAngle,
      Rotation2d hoodAngle,
      double flywheelSpeedRotPerSec) {}

  // Cache parameters
  private ShotParameters latestShot = null;

  private static final double minDistance;
  private static final double maxDistance;
  private static final double phaseDelay;
  private static final InterpolatingTreeMap<Double, Rotation2d> shotHoodAngleMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
  private static final InterpolatingDoubleTreeMap shotFlywheelSpeedMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap timeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  static {
    minDistance = Units.feetToMeters(2 + 2);
    maxDistance = Units.feetToMeters(7 + 2);
    phaseDelay = 0.03; // TODO: untuned
    // These are in degrees from verical
    shotHoodAngleMap.put(Units.feetToMeters(2 + 2), Rotation2d.fromDegrees(22.0));
    shotHoodAngleMap.put(Units.feetToMeters(4 + 2), Rotation2d.fromDegrees(28.0));
    shotHoodAngleMap.put(Units.feetToMeters(6 + 2), Rotation2d.fromDegrees(32.0));
    shotHoodAngleMap.put(Units.feetToMeters(6 + (11 / 12) + 2), Rotation2d.fromDegrees(29.0));
    // shotHoodAngleMap.put(
        // Units.feetToMeters(197.94), Rotation2d.fromDegrees(42.0)); // 147.5 x, 132 y =
    // shotHoodAngleMap.put(Units.feetToMeters(13 + 2), Rotation2d.fromDegrees(44.0));

    // M vs RPS
    shotFlywheelSpeedMap.put(Units.feetToMeters(2 + 2), 53.0);
    shotFlywheelSpeedMap.put(Units.feetToMeters(4 + 2), 60.0);
    shotFlywheelSpeedMap.put(Units.feetToMeters(6 + 2), 66.0);
    shotFlywheelSpeedMap.put(Units.feetToMeters(6 + (11 / 12) + 2), 82.0);
    // shotFlywheelSpeedMap.put(Units.feetToMeters(11 + 2), 72.0);
    // shotFlywheelSpeedMap.put(Units.feetToMeters(13 + 2), 77.0);

    // TODO: Untuned M vs S
    timeOfFlightMap.put(5.68, 1.16);
    timeOfFlightMap.put(4.55, 1.12);
    timeOfFlightMap.put(3.15, 1.11);
    timeOfFlightMap.put(1.88, 1.09);
    timeOfFlightMap.put(1.38, 0.90);
  }

  public ShotParameters calculateShot() {
    if (latestShot != null) {
      return latestShot;
    }

    // Calculate estimated pose while accounting for phase delay
    Pose2d estimatedPose = RobotState.getInstance().getRobotPosition();
    ChassisSpeeds robotRelativeVelocity = RobotState.getInstance().getRobotRelativeVelocity();
    ChassisSpeeds robotVelocity =
        ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeVelocity, estimatedPose.getRotation());
    estimatedPose =
        estimatedPose.exp(
            new Twist2d(
                robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
                robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
                robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay));

    // Calculate distance from turret to target
    Translation2d target = AllianceFlipUtil.apply(RobotState.getInstance().updateGoal().pose);
    Logger.recordOutput("ShotCalculator/Target", new Pose2d(target, Rotation2d.kZero));
    Pose2d turretPosition = estimatedPose.transformBy(robotToTurret);
    double turretToTargetDistance = target.getDistance(turretPosition.getTranslation());

    // Calculate field relative turret velocity
    Rotation2d robotAngle = estimatedPose.getRotation();
    double turretVelocityX =
        robotVelocity.vxMetersPerSecond
            + robotVelocity.omegaRadiansPerSecond
                * (robotToTurret.getY() * robotAngle.getCos()
                    - robotToTurret.getX() * robotAngle.getSin());
    double turretVelocityY =
        robotVelocity.vyMetersPerSecond
            + robotVelocity.omegaRadiansPerSecond
                * (robotToTurret.getX() * robotAngle.getCos()
                    - robotToTurret.getY() * robotAngle.getSin());

    // Account for imparted velocity by robot (turret) to offset
    double timeOfFlight;
    Pose2d lookaheadPose = turretPosition;
    double lookaheadTurretToTargetDistance = turretToTargetDistance;
    for (int i = 0; i < 20; i++) {
      timeOfFlight = timeOfFlightMap.get(lookaheadTurretToTargetDistance);
      double offsetX = turretVelocityX * timeOfFlight;
      double offsetY = turretVelocityY * timeOfFlight;
      lookaheadPose =
          new Pose2d(
              turretPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
              turretPosition.getRotation());
      lookaheadTurretToTargetDistance = target.getDistance(lookaheadPose.getTranslation());
    }

    // Calculate parameters accounted for imparted velocity
    turretAngle =
        target.minus(lookaheadPose.getTranslation()).getAngle().minus(lookaheadPose.getRotation());
    hoodAngle = shotHoodAngleMap.get(lookaheadTurretToTargetDistance);
    latestShot =
        new ShotParameters(
            lookaheadTurretToTargetDistance >= minDistance
                && lookaheadTurretToTargetDistance <= maxDistance,
            turretAngle,
            hoodAngle,
            shotFlywheelSpeedMap.get(lookaheadTurretToTargetDistance));

    // Log calculated values
    Logger.recordOutput("ShotCalculator/LatestShot", latestShot);
    Logger.recordOutput("ShotCalculator/LookaheadPose", lookaheadPose);
    Logger.recordOutput("ShotCalculator/TurretToTargetDistance", lookaheadTurretToTargetDistance);

    return latestShot;
  }

  public void clearCache() {
    latestShot = null;
  }

  @RequiredArgsConstructor
  public enum Goal {
    HUB(FieldConstants.hubCenter),
    LEFT(leftTarget),
    RIGHT(rightTarget),
    CENTER(centerTarget);
    public final Translation2d pose;
  }
}
