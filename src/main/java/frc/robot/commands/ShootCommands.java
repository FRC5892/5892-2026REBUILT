package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.Shooter;
import lombok.AccessLevel;
import lombok.NoArgsConstructor;

@NoArgsConstructor(access = AccessLevel.PRIVATE)
public class ShootCommands {
  public static Command shoot(Indexer indexer, Shooter shooter) {
    return Commands.race(
        indexer.outtake(),
        shooter.getFlywheel().aimCommand(),
        shooter.getHood().aimCommand(),
        shooter.getTurret().aimCommand());
  }
}
