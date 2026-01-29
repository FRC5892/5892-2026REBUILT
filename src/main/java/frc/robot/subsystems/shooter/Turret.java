package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.GenericPositionMechanismSubsystem;
import frc.robot.util.LoggedDIO.LoggedDIO;
import frc.robot.util.LoggedTalon.LoggedTalonFX;
import frc.robot.util.LoggedTunableMeasure;
import frc.robot.util.LoggedTunableNumber;

public class Turret extends GenericPositionMechanismSubsystem {
  public Turret(LoggedTalonFX motor, LoggedDIO reverseLimit, LoggedDIO forwardLimit) {
    super(
        "Turret",
        motor,
        reverseLimit,
        forwardLimit,
        new LoggedTunableNumber("Turret/Homing/Voltage", 4, "v"),
        new LoggedTunableNumber("Turret/Homing/ConfirmVoltage", 4, "v"),
        new LoggedTunableMeasure<>("Turret/Homing/homePosition", Rotations.mutable(0))::get,
        new LoggedTunableMeasure<>("Turret/Homing/homePosition", Rotations.mutable(0.1))::get,
        new LoggedTunableMeasure<>("Turret/Tolerance", Degrees.mutable(5))::get);
    var config =
        new TalonFXConfiguration()
            .withSlot0(new Slot0Configs().withKP(0).withKI(0).withKD(0).withKS(0).withKV(0))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(15)
                    .withMotionMagicAcceleration(30))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(InvertedValue.Clockwise_Positive))
            .withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(5))
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(15));
    motor.withConfig(config).withMMPIDTuning(config);
    setDefaultCommand(aimCommand());
  }

  public Command aimCommand() {
    return run(
        () -> {
          if (homed) {
            this.requestPosition(ShotCalculator.calculateShot().hoodAngle());
          }
        });
  }

  @Override
  protected void periodicUser() {
    ShotCalculator.clearCache();
  }
}
