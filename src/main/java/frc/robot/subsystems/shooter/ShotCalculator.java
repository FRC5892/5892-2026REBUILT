// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants;
import java.util.function.Supplier;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** Thank you so much 6328! */
@RequiredArgsConstructor
public class ShotCalculator {
  private static final Transform2d robotToTurret = new Transform2d();

  public record ShotParameters(
      Rotation2d turretAngle, Rotation2d hoodAngle, AngularVelocity flywheelSpeed) {}

  private static final InterpolatingTreeMap<Double, Rotation2d> shotHoodAngleMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
  private static final InterpolatingDoubleTreeMap shotFlywheelSpeedMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap timeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  static {
    // TODO: tune turret
    //  These are stolen from 6328 for testing

    // These are in degrees from verical
    shotHoodAngleMap.put(1.45, Rotation2d.fromDegrees(19.0));
    shotHoodAngleMap.put(1.75, Rotation2d.fromDegrees(21.0));
    shotHoodAngleMap.put(2.15, Rotation2d.fromDegrees(22.0));
    shotHoodAngleMap.put(2.50, Rotation2d.fromDegrees(23.0));
    shotHoodAngleMap.put(2.84, Rotation2d.fromDegrees(24.0));
    shotHoodAngleMap.put(3.15, Rotation2d.fromDegrees(25.5));
    shotHoodAngleMap.put(3.58, Rotation2d.fromDegrees(26.5));
    shotHoodAngleMap.put(4.16, Rotation2d.fromDegrees(29.0));
    shotHoodAngleMap.put(4.43, Rotation2d.fromDegrees(30.5));
    shotHoodAngleMap.put(5.28, Rotation2d.fromDegrees(34.0));

    shotFlywheelSpeedMap.put(1.45, 175.0);
    shotFlywheelSpeedMap.put(1.75, 185.0);
    shotFlywheelSpeedMap.put(2.15, 190.0);
    shotFlywheelSpeedMap.put(2.50, 200.0);
    shotFlywheelSpeedMap.put(2.84, 210.0);
    shotFlywheelSpeedMap.put(3.15, 218.0);
    shotFlywheelSpeedMap.put(3.58, 222.0);
    shotFlywheelSpeedMap.put(4.16, 230.0);
    shotFlywheelSpeedMap.put(4.43, 235.0);
    shotFlywheelSpeedMap.put(5.28, 250.0);

    timeOfFlightMap.put(1.64227, 0.93);
    timeOfFlightMap.put(2.859544, 1.0);
    timeOfFlightMap.put(4.27071, 1.05);
  }

  private final Supplier<Pose2d> robotPoseSup;
  private final Supplier<ChassisSpeeds> robotVelocitySup;

  @AutoLogOutput(key = "ShotCalculator/Shot")
  private ShotParameters latestShot = null;

  public ShotParameters calculateShot() {
    if (latestShot != null) return latestShot;

    Pose2d robotPose = this.robotPoseSup.get();
    ChassisSpeeds robotVelocity = this.robotVelocitySup.get();

    // Calculate distance from turret to target
    Translation2d target = AllianceFlipUtil.apply(FieldConstants.hubCenter);
    Pose2d turretPosition = robotPose.transformBy(robotToTurret);
    double turretToTargetDistance = target.getDistance(turretPosition.getTranslation());

    // Calculate field relative turret velocity
    double robotAngle = robotPose.getRotation().getRadians();
    double turretVelocityX =
        robotVelocity.vxMetersPerSecond
            + robotVelocity.omegaRadiansPerSecond
                * (robotToTurret.getY() * Math.cos(robotAngle)
                    - robotToTurret.getX() * Math.sin(robotAngle));
    double turretVelocityY =
        robotVelocity.vyMetersPerSecond
            + robotVelocity.omegaRadiansPerSecond
                * (robotToTurret.getX() * Math.cos(robotAngle)
                    - robotToTurret.getY() * Math.sin(robotAngle));

    // Account for imparted velocity by robot (turret) to offset
    double timeOfFlight = timeOfFlightMap.get(turretToTargetDistance);
    double offsetX = turretVelocityX * timeOfFlight;
    double offsetY = turretVelocityY * timeOfFlight;
    Pose2d lookaheadPose =
        new Pose2d(
            turretPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
            turretPosition.getRotation());
    double lookaheadTurretToTargetDistance = target.getDistance(lookaheadPose.getTranslation());

    // Calculate parameters accounted for imparted velocity
    Rotation2d turretAngle = target.minus(lookaheadPose.getTranslation()).getAngle();
    Rotation2d hoodAngle = shotHoodAngleMap.get(lookaheadTurretToTargetDistance);

    latestShot =
        new ShotParameters(
            turretAngle,
            hoodAngle,
            RotationsPerSecond.of(shotFlywheelSpeedMap.get(lookaheadTurretToTargetDistance)));

    // Log calculated values
    Logger.recordOutput("ShotCalculator/LookaheadPose", lookaheadPose);
    Logger.recordOutput("ShotCalculator/TurretToTargetDistance", lookaheadTurretToTargetDistance);
    Logger.recordOutput(
        "ShotCalculator/turretPose",
        new Pose3d(
            new Translation3d(robotPose.getTranslation()).plus(new Translation3d(0, 0, 1)),
            new Rotation3d(
                0, hoodAngle.getRadians() + (3 * Math.PI / 2), turretAngle.getRadians())));

    return latestShot;
  }

  public void clearCache() {
    latestShot = null;
  }
}
