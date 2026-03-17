// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degree;
import static frc.robot.subsystems.vision.VisionConstants.cameraBackName;
import static frc.robot.subsystems.vision.VisionConstants.cameraLeftName;
import static frc.robot.subsystems.vision.VisionConstants.cameraRightName;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ShootCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.indexer.*;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShotCalculator.Goal;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.util.LoggedTalon.TalonFX.NoOppTalonFX;
import frc.robot.util.LoggedTalon.TalonFX.PhoenixTalonFX;
import frc.robot.util.LoggedTalon.TalonFX.TalonFXSimpleMotorSim;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final CANBus rioCAN = new CANBus("rio");
  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private final Intake intake;

  private final Indexer indexer;
  private final Shooter shooter;
  //   private final Led led;

  // Controllers
  private final CommandXboxController driveController = new CommandXboxController(0);
  private final CommandXboxController coDriveController = new CommandXboxController(1);
  private final CommandXboxController testController =
      Constants.tuningMode ? new CommandXboxController(2) : null;

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(cameraBackName, VisionConstants.robotToCameraBack),
                new VisionIOPhotonVision(cameraLeftName, VisionConstants.robotToCameraLeft),
                new VisionIOPhotonVision(cameraRightName, VisionConstants.robotToCameraRight));
        intake =
            new Intake(
                new PhoenixTalonFX(30, rioCAN, "IntakeRoller"),
                new PhoenixTalonFX(31, rioCAN, "IntakeSlapDown"));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIO() {},
                new VisionIO() {},
                new VisionIO() {});
        Logger.recordOutput("Vision/Offsets/" + cameraBackName, VisionConstants.robotToCameraBack);
        Logger.recordOutput("Vision/Offsets/" + cameraLeftName, VisionConstants.robotToCameraLeft);
        Logger.recordOutput(
            "Vision/Offsets/" + cameraRightName, VisionConstants.robotToCameraRight);
        // new Vision(
        //     drive::addVisionMeasurement,
        //     new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose),
        //     new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose));
        intake =
            new Intake(
                new TalonFXSimpleMotorSim(30, rioCAN, "IntakeRoller", 1, 1),
                new TalonFXSimpleMotorSim(31, rioCAN, "IntakeSlap", 1, 1));
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIO() {},
                new VisionIO() {},
                new VisionIO() {});

        intake = new Intake(new NoOppTalonFX("IntakeRoller", 0), new NoOppTalonFX("IntakeSlap", 0));
        break;
    }
    indexer = new Indexer(rioCAN);
    shooter = new Shooter(rioCAN);
    // led = new Led();

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices");

    autoChooser.addOption("Left Auto", Autos.leftAuto(intake, indexer, shooter));
    autoChooser.addOption("Right Auto", Autos.rightAuto(intake, indexer, shooter));
    autoChooser.addOption("Preload Auto", ShootCommands.shoot(indexer, shooter));

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driveController.getLeftY(),
            () -> -driveController.getLeftX(),
            () -> -driveController.getRightX()));

    // Reset gyro to 0° when B button is pressed
    driveController
        .y()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));
    driveController
        .rightBumper()
        .or(coDriveController.rightBumper())
        .whileTrue(ShootCommands.shoot(indexer, shooter));
    driveController.leftBumper().or(coDriveController.leftBumper()).onTrue(intake.intakeSequence());
    driveController.povUp().or(coDriveController.povUp()).onTrue(intake.hold());
    driveController.povLeft().or(coDriveController.povUp()).onTrue(intake.retractForeverCommand());

    driveController
        .povDown()
        .or(coDriveController.povDown())
        .whileTrue(shooter.getHood().stowCommand());

    driveController
        .start()
        .or(coDriveController.start())
        .whileTrue(shooter.homeCommand()); // 8, right middle button, home
    driveController
        .back()
        .or(coDriveController.back())
        .whileTrue(indexer.unjam()); // 7, left middle button, unjam

    driveController
        .x()
        .or(coDriveController.x())
        .onTrue(RobotState.getInstance().setGoalCommand(Goal.LEFT));
    driveController
        .b()
        .or(coDriveController.b())
        .onTrue(RobotState.getInstance().setGoalCommand(Goal.RIGHT));
    driveController
        .a()
        .or(coDriveController.a())
        .onTrue(
            shooter
                .getTurret()
                .forceZero()
                .andThen(shooter.getTurret().gotoPosition(() -> Degree.of(0)))
                .alongWith(Commands.run(() -> {})));
    driveController
        .y()
        .or(coDriveController.y())
        .onTrue(RobotState.getInstance().setGoalCommand(Goal.HUB));

    driveController
        .rightStick()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive, () -> -driveController.getLeftY(), () -> -driveController.getLeftX()));

    if (testController != null) {
      testController
          .x()
          .onTrue(
              shooter
                  .getTurret()
                  .gotoPosition(() -> Degree.of(-120))
                  .alongWith(Commands.run(() -> {})));
      testController
          .a()
          .onTrue(
              shooter
                  .getTurret()
                  .gotoPosition(() -> Degree.of(0))
                  .alongWith(Commands.run(() -> {})));
      testController
          .b()
          .onTrue(
              shooter
                  .getTurret()
                  .gotoPosition(() -> Degree.of(120))
                  .alongWith(Commands.run(() -> {})));
      testController
          .leftBumper()
          .onTrue(shooter.getHood().gotoAngle(() -> Rotation2d.fromDegrees(19)));
      testController.rightBumper().whileTrue(indexer.outtake());
      testController.povUp().whileTrue(shooter.getHood().dutyCycleTestCommand(0.1));
      testController.povDown().whileTrue(shooter.getHood().dutyCycleTestCommand(-0.1));
      testController
          .y()
          .whileTrue(
              shooter.tuneCommand(
                  new LoggedNetworkNumber("/Tuning/SpeedRPS", 0),
                  new LoggedNetworkNumber("/Tuning/HoodAngle", 18.575)));
      testController.start().whileTrue(shooter.homeCommand());
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}

// 21.75
// 0.061 - 0.983
// -358.6 - 32.16
