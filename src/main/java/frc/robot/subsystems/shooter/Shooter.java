package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import frc.robot.Constants;
import frc.robot.util.LoggedDIO.HardwareDIO;
import frc.robot.util.LoggedTalon.Follower.PhoenixTalonFollower;
import frc.robot.util.LoggedTalon.NoOppTalonFX;
import frc.robot.util.LoggedTalon.PhoenixTalonFX;
import frc.robot.util.LoggedTalon.SimpleMotorSim;
import lombok.Getter;

/** Container for shooting bits. This class will initialize the propper IO interfaces. */
public class Shooter {
  @Getter private final Flywheel flywheel;
  @Getter private final Hood hood;
  @Getter private final Turret turret;

  public Shooter(CANBus bus) {
    switch (Constants.currentMode) {
      case REAL -> {
        flywheel =
            new Flywheel(
                new PhoenixTalonFX(
                    25,
                    bus,
                    "Flywheel",
                    new PhoenixTalonFollower(26, MotorAlignmentValue.Aligned)));
        hood =
            new Hood(
                new PhoenixTalonFX(27, bus, "Hood"),
                new HardwareDIO("HoodReverse", 1),
                new HardwareDIO("HoodForward", 2));
        turret =
            new Turret(
                new PhoenixTalonFX(28, bus, "Turret"),
                new HardwareDIO("TurretReverse", 2),
                new HardwareDIO("TurretForward", 3));
      }
      case SIM -> {
        flywheel =
            new Flywheel(
                new SimpleMotorSim(
                    25,
                    bus,
                    "Flywheel",
                    0.0007567661,
                    1 / 1.25,
                    new PhoenixTalonFollower(26, MotorAlignmentValue.Aligned)));
        hood =
            new Hood(
                new SimpleMotorSim(27, bus, "Hood", 0.0017154536, 1.3),
                new HardwareDIO("HoodReverse", 1),
                new HardwareDIO("HoodForward", 2));
        turret =
            new Turret(
                new SimpleMotorSim(28, bus, "Turret", 0.0307668163, 1.25),
                new HardwareDIO("TurretReverse", 2),
                new HardwareDIO("TurretForward", 3));
      }
      default -> {
        flywheel = new Flywheel(new NoOppTalonFX("Flywheel", 1));
        hood =
            new Hood(
                new NoOppTalonFX("Hood", 0),
                new HardwareDIO("HoodReverse", 1),
                new HardwareDIO("HoodForward", 2));
        turret =
            new Turret(
                new NoOppTalonFX("Turret", 0),
                new HardwareDIO("TurretReverse", 2),
                new HardwareDIO("TurretForward", 3));
      }
    }

    hood.stowCommand();
  }
}
