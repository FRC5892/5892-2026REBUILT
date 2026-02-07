// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Rotation;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTalon.TalonFX.LoggedTalonFX;
import frc.robot.util.LoggedTunableMeasure;
import frc.robot.util.LoggedTunableNumber;

public class Intake extends SubsystemBase {
  private final LoggedTalonFX rollerMotor;
  private final LoggedTalonFX slapDownMotor;
  private final LoggedTunableNumber tunedVoltage =
      new LoggedTunableNumber("Intake/RollerVoltage", 2);
  private final LoggedTunableMeasure<MutAngle> outPosition =
      new LoggedTunableMeasure<>("Intake/OutPosition", Rotation.mutable(0.5));
  private final LoggedTunableMeasure<MutAngle> inPosition =
      new LoggedTunableMeasure<>("Intake/InPosition", Rotation.mutable(0));
  private final LoggedTunableMeasure<MutAngle> tolerance =
      new LoggedTunableMeasure<>("Intake/Tolerance", Rotation.mutable(0.05));

  private final VoltageOut voltageOut = new VoltageOut(tunedVoltage.get()).withEnableFOC(true);
  private final MotionMagicTorqueCurrentFOC mmOut = new MotionMagicTorqueCurrentFOC(0);

  /** Creates a new Intake. */
  public Intake(LoggedTalonFX rollerMotor, LoggedTalonFX slapDownMotor) {
    this.rollerMotor = rollerMotor.withConfig(LoggedTalonFX.buildStandardConfig(20, 20));
    var slapDownConfig =
        LoggedTalonFX.buildStandardConfig(20, 20, NeutralModeValue.Brake)
            .withSlot0(new Slot0Configs().withKP(0).withKI(0).withKD(0).withKS(0).withKV(0))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicAcceleration(2)
                    .withMotionMagicCruiseVelocity(5));
    this.slapDownMotor = slapDownMotor.withConfig(slapDownConfig).withMMPIDTuning(slapDownConfig);
  }

  public Command intakeCommand() {
    return startEnd(
        () -> rollerMotor.setControl(voltageOut.withOutput(tunedVoltage.get())),
        () -> rollerMotor.setControl(voltageOut.withOutput(0)));
  }

  public Command extendCommand() {
    return startEnd(
            () -> slapDownMotor.setControl(mmOut.withPosition(this.outPosition.get())), () -> {})
        .until(() -> slapDownMotor.atSetpoint(outPosition.get(), tolerance.get()));
  }

  public Command retractCommand() {
    return startEnd(
            () -> slapDownMotor.setControl(mmOut.withPosition(this.inPosition.get())), () -> {})
        .until(() -> slapDownMotor.atSetpoint(inPosition.get(), tolerance.get()));
  }

  public Command intakeSequence() {
    return extendCommand().andThen(intakeCommand());
  }

  // Everywhere constant is referenced use tunable number instead

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    rollerMotor.periodic();
    slapDownMotor.periodic();
  }
}
