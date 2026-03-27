// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTalon.TalonFX.LoggedTalonFX;
import frc.robot.util.LoggedTunableMeasure;

public class Intake extends SubsystemBase {
  private final LoggedTalonFX rollerMotor;
  private final LoggedTalonFX slapDownMotor;
  private final LoggedTunableMeasure<MutAngularVelocity> rollerVelocity =
      new LoggedTunableMeasure<>("Intake/RollerVelocity", RotationsPerSecond.mutable(15));
  private final LoggedTunableMeasure<MutAngle> outPosition =
      new LoggedTunableMeasure<>("Intake/OutPosition", Rotation.mutable(0.49));
  private final LoggedTunableMeasure<MutAngle> holdPosition =
      new LoggedTunableMeasure<>("Intake/HoldPosition", Rotation.mutable(0.32));
  private final LoggedTunableMeasure<MutAngularVelocity> extendOuttakeVelocity =
      new LoggedTunableMeasure<>("Intake/ExtendOuttakeVelocity", RotationsPerSecond.mutable(-4));
  private final LoggedTunableMeasure<MutAngle> inPosition =
      new LoggedTunableMeasure<>("Intake/InPosition", Rotation.mutable(0.02));
  private final LoggedTunableMeasure<MutAngle> tolerance =
      new LoggedTunableMeasure<>("Intake/Tolerance", Rotation.mutable(0.05));

  private final VelocityTorqueCurrentFOC rollerVelocityOut = new VelocityTorqueCurrentFOC(0);
  private final MotionMagicDutyCycle mmOut = new MotionMagicDutyCycle(0);

  /** Creates a new Intake. */
  public Intake(LoggedTalonFX rollerMotor, LoggedTalonFX slapDownMotor) {
    var rollerConfig =
        LoggedTalonFX.buildStandardConfig(160, 40)
            .withTorqueCurrent(
                new TorqueCurrentConfigs()
                    .withPeakForwardTorqueCurrent(80)
                    .withPeakReverseTorqueCurrent(80))
            .withSlot0(
                new Slot0Configs()
                    .withKP(0.8)
                    .withKI(0)
                    .withKD(0)
                    .withKS(0.25)
                    .withKV(0)
                    .withKA(0));
    this.rollerMotor = rollerMotor.withConfig(rollerConfig).withPIDTunable(rollerConfig.Slot0);
    var slapDownConfig =
        LoggedTalonFX.buildStandardConfig(80, 4020)
            .withSlot0(new Slot0Configs().withKP(10).withKI(0).withKD(0).withKS(0).withKV(0))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicAcceleration(5)
                    .withMotionMagicCruiseVelocity(8))
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(23 * (36 / 24)));
    this.slapDownMotor = slapDownMotor.withConfig(slapDownConfig).withMMPIDTuning(slapDownConfig);

    // setDefaultCommand(retractForeverCommand());
  }

  public Command intakeCommand() {
    return startEnd(
        () -> rollerMotor.setControl(rollerVelocityOut.withVelocity(rollerVelocity.get())),
        () -> rollerMotor.setControl(rollerVelocityOut.withVelocity(0)));
  }

  public Command extendCommand() {
    return startEnd(
            () -> {
              slapDownMotor.setControl(mmOut.withPosition(this.outPosition.get()));
              rollerMotor.setControl(rollerVelocityOut.withVelocity(extendOuttakeVelocity.get()));
            },
            () -> {
              rollerMotor.setControl(rollerVelocityOut.withVelocity(0));
            })
        .until(() -> slapDownMotor.atSetpoint(outPosition.get(), tolerance.get()));
  }

  public Command retractForeverCommand() {
    return startEnd(
            () -> slapDownMotor.setControl(mmOut.withPosition(this.inPosition.get())), () -> {})
        .until(() -> slapDownMotor.atSetpoint(inPosition.get(), tolerance.get()));
  }

  public Command retractCommand() {
    return startEnd(
        () -> slapDownMotor.setControl(mmOut.withPosition(this.inPosition.get())), () -> {});
  }

  public Command intakeSequence() {
    return extendCommand().andThen(intakeCommand());
  }

  public Command hold() {
    return startRun(
        () -> slapDownMotor.setControl(mmOut.withPosition(holdPosition.get())), () -> {});
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    rollerMotor.periodic();
    slapDownMotor.periodic();
  }
}
