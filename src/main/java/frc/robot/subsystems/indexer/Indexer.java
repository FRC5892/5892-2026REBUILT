package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.util.LoggedDIO.HardwareDIO;
import frc.robot.util.LoggedDIO.NoOppDio;
import frc.robot.util.LoggedDIO.SimDIO;
import frc.robot.util.LoggedTalon.TalonFX.NoOppTalonFX;
import frc.robot.util.LoggedTalon.TalonFX.PhoenixTalonFX;
import frc.robot.util.LoggedTalon.TalonFX.TalonFXFlywheelSim;
import frc.robot.util.LoggedTalon.TalonFX.TalonFXSimpleMotorSim;
import frc.robot.util.RollerSubsystem.Direction;
import lombok.Getter;

/**
 * Container for indexing and ball manipulation bits This class will initialize the propper IO
 * interfaces.
 */
public class Indexer {
  @Getter private final Kicker kicker;
  @Getter private final Rollers rollers;
  @Getter private final Spindexer spindexer;

  public Indexer(CANBus canBus) {
    switch (Constants.currentMode) {
      case REAL -> {
        kicker = new Kicker(new PhoenixTalonFX(20, canBus, "Kicker"));
        rollers =
            new Rollers(
                new PhoenixTalonFX(21, canBus, "Rollers"), new HardwareDIO("RollerBeam", 0));
        spindexer = new Spindexer(new PhoenixTalonFX(22, canBus, "Spindexer"));
      }
      case SIM -> {
        kicker = new Kicker(new TalonFXFlywheelSim(20, canBus, "Kicker", 0.5, 1));
        rollers =
            new Rollers(
                new TalonFXSimpleMotorSim(21, canBus, "Rollers", 0.5, 1),
                new SimDIO("RollerBeam", SimDIO.fromNT("RollerBeam")));
        spindexer = new Spindexer(new TalonFXSimpleMotorSim(22, canBus, "Spindexer", 0.5, 1));
      }
      default -> {
        kicker = new Kicker(new NoOppTalonFX("Kicker", 0));
        rollers = new Rollers(new NoOppTalonFX("Rollers", 0), new NoOppDio("RollerBeam"));
        spindexer = new Spindexer(new NoOppTalonFX("Spindexer", 0));
      }
    }
  }

  public Command outtake() {
    return Commands.parallel(
        kicker.runRoller(Direction.FORWARD),
        rollers.runRoller(Direction.FORWARD),
        spindexer.runRoller(Direction.FORWARD));
  }

  public Command unjam() {
    return Commands.parallel(
        kicker.runRoller(Direction.REVERSE),
        rollers.runRoller(Direction.REVERSE),
        spindexer.runRoller(Direction.REVERSE));
  }

  public Command prepare() {
    return Commands.parallel(
            rollers.runRoller(Direction.FORWARD), spindexer.runRoller(Direction.FORWARD))
        .until(rollers::beamBroken);
  }
}
