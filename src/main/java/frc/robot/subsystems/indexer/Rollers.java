package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.util.LoggedDIO.LoggedDIO;
import frc.robot.util.LoggedTalon.TalonFX.LoggedTalonFX;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.RollerSubsystem;

public class Rollers extends RollerSubsystem {
  private final LoggedDIO beambreak;

  public Rollers(LoggedTalonFX motor, LoggedDIO beambreak) {
    super(
        motor.withConfig(LoggedTalonFX.buildStandardConfig(40, 40, NeutralModeValue.Brake)),
        new LoggedTunableNumber("Rollers/Speed", 0.5),
        new LoggedTunableNumber("Rollers/UnjamSpeed", 0.25));
    this.beambreak = beambreak.withReversed(true);
  }

  public boolean beamBroken() {
    return beambreak.get();
  }
}
