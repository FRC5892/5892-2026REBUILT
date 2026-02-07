package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.util.LoggedTalon.TalonFX.LoggedTalonFX;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.RollerSubsystem;

public class Spindexer extends RollerSubsystem {
  public Spindexer(LoggedTalonFX motor) {
    super(
        motor.withConfig(LoggedTalonFX.buildStandardConfig(40, 40, NeutralModeValue.Brake)),
        new LoggedTunableNumber("Spindexer/Speed", 0.5),
        new LoggedTunableNumber("Spindexer/UnjamSpeed", 0.25));
  }
}
