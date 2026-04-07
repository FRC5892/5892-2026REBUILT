package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotation;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.util.LoggedAnalogInput.LoggedAnalogInput;
import frc.robot.util.LoggedAnalogInput.SimAnalogInput;
import frc.robot.util.LoggedDIO.LoggedDIO;
import frc.robot.util.LoggedTalon.TalonFX.LoggedTalonFX;
import frc.robot.util.LoggedTunableMeasure;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.MechanismUtil;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class Turret extends SubsystemBase {

  /* Hardware */
  private final LoggedTalonFX motor;
  private final LoggedDIO forwardLimit;
  private final LoggedAnalogInput pot;

  /* Movement Constants */
  private final LoggedTunableMeasure<MutAngle> minAngle =
      new LoggedTunableMeasure<MutAngle>("Turret/MinAngle", Degrees.mutable(-202));
  private final LoggedTunableMeasure<MutAngle> maxAngle =
      new LoggedTunableMeasure<MutAngle>("Turret/MaxAngle", Degrees.mutable(205));
  /* Homing */
  private final LoggedTunableNumber homingVoltage =
      new LoggedTunableNumber("Turret/Homing/Voltage", 1.5, "v");
  private final LoggedTunableNumber homingConfirmationVoltage =
      new LoggedTunableNumber("Turret/Homing/ConfirmVoltage", 0.5, "v");
  private final LoggedTunableMeasure<MutAngle> homingSwitchPosition =
      new LoggedTunableMeasure<>("Turret/Homing/HomePosition", Degrees.mutable(205));
  private final LoggedTunableMeasure<MutAngle> homingConfirmPosition =
      new LoggedTunableMeasure<>("Turret/Homing/ConfirmPosition", Degrees.mutable(195));
  private final LoggedTunableMeasure<MutAngle> tolerance =
      new LoggedTunableMeasure<>("Turret/Tolerance", Degrees.mutable(5));

  /**
   * Pose of the Potentiometer at 0 degrees This is not forward, this is halfway between limits
   * MaxRotation - MinRotation MaxRotation/MinRotation: Motor/Turret/PositionRot (converted to
   * degrees) at each turret limit
   */
  private final LoggedTunableMeasure<MutAngle> pot0Pose =
      new LoggedTunableMeasure<MutAngle>("Turret/Pot/0Pose", Degrees.mutable(206.885272));

  /**
   * Range of the Potentiometer: | MaxRotation - MinRotation | ----------------------------- |
   * MaxPercent - MinPercent |
   *
   * <p>MaxRotation/MinRotation: Motor/Turret/PositionRot (converted to degrees) at each turret
   * limit MaxPercent/MinPercent: DigitalInput/TurretPot/Value at each limit
   */
  private final LoggedTunableMeasure<MutAngle> potRange =
      new LoggedTunableMeasure<MutAngle>("Turret/Pot/Range", Degrees.mutable(422.057991062));

  private final LoggedTunableMeasure<MutAngle> wrapWarningThreshold =
      new LoggedTunableMeasure<>("Turret/WrapWarningThreshold", Degrees.mutable(10));

  /** Difference between Turret 0 and robot forward. This needs to be fairly precise */
  private final LoggedTunableMeasure<MutAngle> offset =
      new LoggedTunableMeasure<MutAngle>("Turret/Offset", Degrees.mutable(65));

  private final LoggedTunableMeasure<MutAngle> staticPose =
      new LoggedTunableMeasure<MutAngle>("Turret/StaticPose", Degrees.mutable(0));
  /* Control Requests */
  private final MotionMagicDutyCycle mmControl = new MotionMagicDutyCycle(0);
  private final NeutralOut neutralControl = new NeutralOut();

  /* State */
  private final MutAngle targetPosition = Degrees.mutable(0);
  @AutoLogOutput private final MutAngle potPose = Degrees.mutable(0);
  private boolean positionControl = false;
  @Getter @Setter private boolean homed = false;
  @Getter private boolean atSetpoint = false;

  private final LoggedNetworkBoolean estopFlag =
      new LoggedNetworkBoolean("SmartDashboard/TurretEStop", false);
  private final LoggedNetworkBoolean holdFlag =
      new LoggedNetworkBoolean("SmartDashboard/TurretHold", false);

  private final LoggedNetworkBoolean staticFlag =
      new LoggedNetworkBoolean("SmartDashboard/TurretStatic", false);

  private Angle holdOffset = null;
  private final CurrentLimitsConfigs currentConfig;

  private final Translation3d turretVisual = new Translation3d(0, 0, Units.inchesToMeters(20));

  public Turret(LoggedTalonFX motor, LoggedDIO reverseLimit, LoggedAnalogInput pot) {
    this.motor = motor;
    this.forwardLimit = reverseLimit.withReversed(false);
    this.pot = new SimAnalogInput("Turret/Pot", () -> 0.0);

    var config =
        new TalonFXConfiguration()
            .withSlot0(
                new Slot0Configs().withKP(10).withKI(0).withKD(0).withKS(0.018).withKV(0.085))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(1)
                    .withMotionMagicAcceleration(20))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(InvertedValue.CounterClockwise_Positive))
            .withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(20))
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(44));
    this.currentConfig = config.CurrentLimits;
    // primary: 12:48, 4:1
    // secondary: 10:110, 11:1
    // total: 44:1
    motor.withConfig(config).withMMPIDTuning(SlotConfigs.from(config.Slot0), config.MotionMagic);
    if (Constants.tuningMode) {
      // This command directly sets the turret position to 0. It really should never
      // be used ever
      // ever
      SmartDashboard.putData("Turret/ForceZero", forceZero());
      SmartDashboard.putData("Turret/UnHome", runOnce(() -> setHomed(false)));
    }
    // Preload so AdvantageKit can process logging stuff before the match starts.
    ShotCalculator.getInstance().calculateShot();
    // Home if not already homed
    RobotModeTriggers.disabled().negate().and(() -> !this.isHomed()).onTrue(homingCommand());
    setDefaultCommand(aimCommand());
  }

  public Command aimCommand() {
    return run(() -> {
          if (!homed) return;
          if (estopFlag.get()) {
            positionControl = false;
            motor.setControl(neutralControl);
            return;
          }
          if (holdFlag.get()) {
            if (holdOffset == null) {
              holdOffset = motor.getPosition();
            }
            requestPosition(staticPose.get().copy().plus(holdOffset));
            return;
          }
          holdOffset = null;
          if (staticFlag.get()) {
            this.requestPosition(staticPose.get());
            return;
          }
          this.requestPosition(
              Radians.of(
                  MathUtil.angleModulus(
                      ShotCalculator.getInstance().calculateShot().turretAngle().getRadians()
                          + offset.get().baseUnitMagnitude())));
        })
        .withName("Turret Aim");
  }

  public Command homingCommand() {
    return MechanismUtil.buildHomingCommand(
            motor,
            forwardLimit,
            this,
            homingVoltage,
            true,
            homingSwitchPosition::get,
            homingConfirmationVoltage::get,
            homingConfirmPosition::get)
        .beforeStarting(
            () -> {
              positionControl = false;
              motor.quickApplyConfig(this.currentConfig.clone().withStatorCurrentLimit(10));
            })
        .finallyDo(
            (i) -> {
              if (!i) {
                setHomed(true);
              }
              motor.quickApplyConfig(this.currentConfig);
            })
        .onlyIf(() -> !estopFlag.get());
  }

  public void updateFromAbsolute() {
    motor.setPosition(potPose);
    this.homed = true;
  }

  public Command updateFromAbsoluteCommand() {
    return runOnce(this::updateFromAbsolute).ignoringDisable(true);
  }

  /**
   * A command that requests the turret to move to a position. The command completes imminently,
   * without waiting for a tolerance to be achieved.
   *
   * @param position a supplier of the target position
   * @return the command
   */
  public Command requestPosition(Supplier<Angle> position) {
    return runOnce(
        () -> {
          requestPosition(position.get());
        });
  }

  public void requestPosition(Angle position) {

    targetPosition.mut_setBaseUnitMagnitude(
        MathUtil.inputModulus(
            position.baseUnitMagnitude(),
            minAngle.get().baseUnitMagnitude(),
            maxAngle.get().baseUnitMagnitude()));
    positionControl = true;
    setControl();
  }

  /**
   * A command that commands the Turret to move to a position. This command ends when the setpoint
   * is archived
   *
   * @param position a supplier of the target position
   * @return the command
   */
  public Command gotoPosition(Supplier<Angle> position) {
    // I really shouldn't but by creating a functional command I don't create 5
    // extra objects by
    // separating this out.
    return new FunctionalCommand(
        () -> requestPosition(position.get()),
        () -> {}, // Nothing to do periodically. Motion is controlled in the periodic function
        (i) -> {},
        () -> atSetpoint,
        this);
  }

  public Command forceZero() {
    return runOnce(
            () -> {
              motor.setPosition(Degrees.of(0));
              this.setHomed(true);
            })
        .ignoringDisable(true);
  }

  @Override
  public final void periodic() {
    motor.periodic();
    forwardLimit.periodic();
    pot.periodic();

    potPose.mut_setBaseUnitMagnitude(
        (pot.get() * potRange.get().baseUnitMagnitude()) - pot0Pose.get().baseUnitMagnitude());

    atSetpoint = motor.atSetpoint(targetPosition, tolerance.get());
    Logger.recordOutput("Turret/AtSetpoint", atSetpoint);
    Logger.recordOutput("Turret/Homed", homed);
    Logger.recordOutput("Turret/PositionControl", positionControl);

    double pos = motor.getPosition().baseUnitMagnitude();
    boolean nearWrap =
        Math.abs(pos - minAngle.get().baseUnitMagnitude())
                < wrapWarningThreshold.get().baseUnitMagnitude()
            || Math.abs(pos - maxAngle.get().baseUnitMagnitude())
                < wrapWarningThreshold.get().baseUnitMagnitude();
    RobotState.getInstance().setTurretNearWrapPoint(nearWrap);

    setControl();
    Logger.recordOutput(
        "Turret/VisibleSetpoint",
        new Pose3d(RobotState.getInstance().getRobotPosition())
            .transformBy(
                new Transform3d(
                    turretVisual, new Rotation3d(Degrees.zero(), Degrees.zero(), targetPosition))));

    RobotState.getInstance().setTurretAtSetpoint(atSetpoint);
  }

  private void setControl() {
    if (positionControl) {
      Logger.recordOutput("Turret/Target", targetPosition.in(Rotation), "rot");
      motor.setControl(
          mmControl.withPosition(targetPosition).withLimitForwardMotion(forwardLimit.get()));
    }
  }
}
