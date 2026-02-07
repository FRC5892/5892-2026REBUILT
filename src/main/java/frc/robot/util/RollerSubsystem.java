package frc.robot.util;

import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTalon.TalonFX.LoggedTalonFX;
import java.util.function.DoubleSupplier;
import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor
public class RollerSubsystem extends SubsystemBase {
  protected final LoggedTalonFX motor;
  protected final DoubleSupplier forwardVolts;
  protected final DoubleSupplier reverseVolts;
  protected final VoltageOut voltageOut = new VoltageOut(0).withEnableFOC(true);

  public RollerSubsystem(LoggedTalonFX motor, DoubleSupplier forwardVolts) {
    // Use the same speed for both directions
    this(motor, forwardVolts, forwardVolts);
  }

  @Override
  public void periodic() {
    motor.periodic();
  }

  public Command runRoller(DoubleSupplier speed) {
    return runEnd(
        () -> motor.setControl(voltageOut.withOutput(speed.getAsDouble())),
        () -> motor.setControl(voltageOut.withOutput(0)));
  }

  public Command runRoller(Direction direction) {
    DoubleSupplier supplier =
        switch (direction) {
          case FORWARD -> forwardVolts;
          case REVERSE -> reverseVolts;
        };
    return runRoller(supplier);
  }

  public Command stop() {
    return startRun(() -> motor.setControl(voltageOut.withOutput(0)), () -> {});
  }

  public enum Direction {
    FORWARD,
    REVERSE
  }
}
