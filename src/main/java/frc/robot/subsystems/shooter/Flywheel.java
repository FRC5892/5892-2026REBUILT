package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTalon.TalonFX.LoggedTalonFX;
import frc.robot.util.LoggedTunableMeasure;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class Flywheel extends SubsystemBase {

  private final LoggedTalonFX motor;

  private final VelocityTorqueCurrentFOC control = new VelocityTorqueCurrentFOC(0);
  @Getter @AutoLogOutput private boolean atSetpoint = false;
  private final LoggedTunableMeasure<MutAngularVelocity> tolerance =
      new LoggedTunableMeasure<>("Flywheel/Tolerance", RotationsPerSecond.mutable(5));
  private final LoggedTunableMeasure<MutAngularVelocity> staticSpeed =
      new LoggedTunableMeasure<>("Flywheel/staticSpeed", RotationsPerSecond.mutable(60));

  private final LoggedNetworkBoolean estopFlag =
      new LoggedNetworkBoolean("SmartDashboard/FlywheelEStop", false);
  private final LoggedNetworkBoolean staticFlag =
      new LoggedNetworkBoolean("SmartDashboard/FlywheelStatic", false);

  public Flywheel(LoggedTalonFX motor) {
    this.motor = motor;
    var config =
        LoggedTalonFX.buildStandardConfig(80, 60)
            .withTorqueCurrent(
                new TorqueCurrentConfigs()
                    .withPeakForwardTorqueCurrent(40)
                    .withPeakReverseTorqueCurrent(0))
            .withMotorOutput(
                new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive))
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(24 / 30))
            .withSlot0(
                new Slot0Configs()
                    .withKP(5)
                    .withKI(0)
                    .withKD(0)
                    .withKS(1.5)
                    .withKV(0.05)
                    .withKA(0));
    motor.withConfig(config).withPIDTunable(config.Slot0);
    setDefaultCommand(aimCommand());
  }

  public void setSetpoint(AngularVelocity velocity) {
    motor.setControl(control.withVelocity(velocity));
  }

  public Command setpointTestCommand(DoubleSupplier velocityRPS) {
    return runEnd(
        () -> motor.setControl(control.withVelocity(velocityRPS.getAsDouble())),
        () -> motor.setControl(control.withVelocity(0)));
  }

  public Command aimCommand() {
    return run(
        () -> {
          if (estopFlag.get()) {
            motor.setControl(control.withVelocity(0));
            return;
          }
          if (staticFlag.get()) {
            motor.setControl(control.withVelocity(staticSpeed.get()));
            return;
          }
          setSetpoint(
              RotationsPerSecond.of(
                  ShotCalculator.getInstance().calculateShot().flywheelSpeedRotPerSec()));
        });
  }

  @Override
  public void periodic() {
    motor.periodic();
    atSetpoint = motor.atSetpoint(control.getVelocityMeasure(), tolerance.get());
    ShotCalculator.getInstance().clearCache();
  }
}
