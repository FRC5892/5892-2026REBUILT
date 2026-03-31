// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Rotation;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTalon.TalonFX.LoggedTalonFX;
import frc.robot.util.LoggedTunableMeasure;
import frc.robot.util.LoggedTunableNumber;

public class Intake extends SubsystemBase {
  private final LoggedTalonFX rollerMotor;
  private final LoggedTalonFX slapDownMotor;
  private final LoggedTunableNumber rollerSpeed =
      new LoggedTunableNumber("IntakeRoller/Speed", 0.9, "%");
  private final LoggedTunableNumber unjamThreshold =
      new LoggedTunableNumber("IntakeRoller/UnjamThreshold", 100, "amp");
  private final LoggedTunableNumber unjamSpeed =
      new LoggedTunableNumber("IntakeRoller/UnjamSpeed", -0.2, "%");
  private final LoggedTunableMeasure<MutAngle> outPosition =
      new LoggedTunableMeasure<>("IntakeSlap/OutPosition", Rotation.mutable(0.49));
  private final LoggedTunableMeasure<MutAngle> holdPosition =
      new LoggedTunableMeasure<>("IntakeSlap/HoldPosition", Rotation.mutable(0.32));
  private final LoggedTunableNumber extendOuttakeSpeed =
      new LoggedTunableNumber("IntakeRoller/ExtendOuttakeSpeed", -0.1);
  private final LoggedTunableMeasure<MutAngle> inPosition =
      new LoggedTunableMeasure<>("IntakeSlap/InPosition", Rotation.mutable(0.02));
  private final LoggedTunableMeasure<MutAngle> tolerance =
      new LoggedTunableMeasure<>("IntakeSlap/Tolerance", Rotation.mutable(0.05));

  private final DutyCycleOut dutyCycleOut = new DutyCycleOut(rollerSpeed.get()).withEnableFOC(true);
  private final MotionMagicDutyCycle mmOut = new MotionMagicDutyCycle(0);

  /** Creates a new Intake. */
  public Intake(LoggedTalonFX rollerMotor, LoggedTalonFX slapDownMotor) {
    this.rollerMotor = rollerMotor.withConfig(LoggedTalonFX.buildStandardConfig(160, 60));
    var slapDownConfig =
        LoggedTalonFX.buildStandardConfig(80, 4020)
            .withSlot0(new Slot0Configs().withKP(10).withKI(0).withKD(0).withKS(0).withKV(0))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicAcceleration(5)
                    .withMotionMagicCruiseVelocity(8))
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(23 * (36.0 / 24.0)));
    this.slapDownMotor = slapDownMotor.withConfig(slapDownConfig).withMMPIDTuning(slapDownConfig);

    // setDefaultCommand(retractForeverCommand());
  }

  public Command intakeCommand() {
    return runEnd(
        () ->
            rollerMotor.setControl(
                dutyCycleOut.withOutput(
                    // rollerMotor.getPrimaryTorqueCurrentAmps() > unjamThreshold.get() ? unjamSpeed.get() :
                      rollerSpeed.get())),
        () -> rollerMotor.setControl(dutyCycleOut.withOutput(0)));
  }

  public Command extendCommand() {
    return startEnd(
            () -> {
              slapDownMotor.setControl(mmOut.withPosition(this.outPosition.get()));
              rollerMotor.setControl(dutyCycleOut.withOutput(extendOuttakeSpeed.get()));
            },
            () -> {
              rollerMotor.setControl(dutyCycleOut.withOutput(0));
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
