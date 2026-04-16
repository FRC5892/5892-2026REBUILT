// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShotCalculator.Goal;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class Autos {

  public static Command leftAuto(Intake intake, Indexer indexer, Shooter shooter) {
    try {
      final ArrayList<PathPoint> points;
      if (Constants.currentMode == Constants.Mode.SIM) {
        points = new ArrayList<>();
      } else {
        points = null;
      }
      // Boilerplate is over. Now do the actual logic
      final Command auto =
          Commands.sequence(
              Commands.race(
                  Commands.sequence(
                      AutoBuilder.followPath(loadPath("Left_Path_Start", points)),
                      AutoBuilder.followPath(loadPath("Left_Path_Return", points))),
                  intake.intakeSequence(),
                  shooter.getHood().stowCommand()),
              shooter.getTurret().homingCommand(),
              Commands.race(
                  ShootCommands.shoot(indexer, shooter, Goal.HUB),
                  Commands.waitSeconds(7),
                  Commands.sequence(Commands.waitSeconds(3.5), intake.hold())),
                  Commands.race(
                      Commands.sequence(
                          AutoBuilder.followPath(loadPath("Left_Path", points)),
                          AutoBuilder.followPath(loadPath("Left_Path_Return", points))),
                      intake.intakeSequence(),
                      shooter.getHood().stowCommand()),
                  Commands.race(
                      ShootCommands.shoot(indexer, shooter, Goal.HUB),
                      Commands.waitSeconds(7),
                      Commands.sequence(Commands.waitSeconds(3.5), intake.hold())));
      // More boilerplate
      if (Constants.currentMode == Constants.Mode.SIM) {
        Logger.recordOutput(
            "Autos/LeftAuto", points.stream().map(m -> m.position).toArray(Translation2d[]::new));
      }
      return auto;
    } catch (Exception e) {
      @SuppressWarnings("resource")
      Alert alert = new Alert("Failed to load Left Auto", AlertType.kError);
      alert.set(true);
      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
      return Commands.none();
    }
  }

  public static Command rightAuto(Intake intake, Indexer indexer, Shooter shooter) {
    try {
      final ArrayList<PathPoint> points;
      if (Constants.currentMode == Constants.Mode.SIM) {
        points = new ArrayList<>();
      } else {
        points = null;
      }
      // Boilerplate is over. Now do the actual logic
      final Command auto =
          Commands.sequence(
              Commands.race(
                  Commands.sequence(
                      AutoBuilder.followPath(loadPath("Right_Path_Start", points)),
                      AutoBuilder.followPath(loadPath("Right_Path_Return", points))),
                  intake.intakeSequence(),
                  shooter.getHood().stowCommand()),
              shooter.getTurret().homingCommand(),
              Commands.race(
                  ShootCommands.shoot(indexer, shooter, Goal.HUB),
                  Commands.waitSeconds(7),
                  Commands.sequence(Commands.waitSeconds(3.5), intake.hold())),
              Commands.race(
                  Commands.sequence(
                      AutoBuilder.followPath(loadPath("Right_Path", points)),
                      AutoBuilder.followPath(loadPath("Right_Path_Return", points))),
                  intake.intakeSequence(),
                  shooter.getHood().stowCommand()),
              Commands.race(
                  ShootCommands.shoot(indexer, shooter, Goal.HUB),
                  Commands.waitSeconds(7),
                  Commands.sequence(Commands.waitSeconds(3.5), intake.hold())));
      // More boilerplate
      if (Constants.currentMode == Constants.Mode.SIM) {
        Logger.recordOutput(
            "Autos/RightAuto", points.stream().map(m -> m.position).toArray(Translation2d[]::new));
      }
      return auto;
    } catch (Exception e) {
      @SuppressWarnings("resource")
      Alert alert = new Alert("Failed to load Right Auto", AlertType.kError);
      alert.set(true);
      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
      return Commands.none();
    }
  }

  public static Command preload(Indexer indexer, Shooter shooter) {
    return ShootCommands.shoot(indexer, shooter).withTimeout(7);
  }

  public static PathPlannerPath loadPath(String name, List<PathPoint> points)
      throws IOException, ParseException {
    var path = PathPlannerPath.fromPathFile(name);
    if (points != null) {
      points.addAll(path.getAllPathPoints());
    }
    return path;
  }

  private static Command loadLogFollow(String name, List<PathPoint> points)
      throws IOException, ParseException {
    return AutoBuilder.followPath(loadPath(name, points)).withName(name);
  }
}
