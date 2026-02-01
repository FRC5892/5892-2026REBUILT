package frc.robot.util.LoggedTalon;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
// import frc.robot.util.LoggedTalon.Follower.TalonFXFollower;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.Function;
import org.littletonrobotics.junction.Logger;

/**
 * A TalonFX that is logged. Construct {@link PhoenixTalonFX}, {@link BaseTalonFXSim}, or {@link
 * NoOppTalonFX} to use this class.
 */
public abstract class LoggedTalonFX {
  protected final String name;
  private final TalonInputsAutoLogged inputs = new TalonInputsAutoLogged();
  private final Alert[] connectionAlerts;
  private boolean pidTuning = false;
  private boolean mmTuning = false;

  private LoggedTunableNumber[] tunableNumbers = null;
  private SlotConfigs tunedConfigs = null;
  private MotionMagicConfigs mmTunedConfigs = null;
  private LoggedTalonFX[] tuningFollowers = null;
  protected final int followers;

  private final MutAngularVelocity velocity = RadiansPerSecond.mutable(0);
  private final MutAngle position = Radian.mutable(0);

  public LoggedTalonFX(String name, int followers) {
    this.followers = followers;
    this.name = name;
    this.connectionAlerts = new Alert[followers + 1];
    this.connectionAlerts[0] =
        new Alert("TalonFX" + name + " is not connected", Alert.AlertType.kError);
    if (followers != 0) {
      for (int i = 1; i <= followers; i++) {
        connectionAlerts[i] =
            new Alert(
                "TalonFX " + name + " follower " + i + " is not connected", Alert.AlertType.kError);
      }
    }
    inputs.torqueCurrentAmps = new double[followers + 1];
    inputs.temperatureC = new double[followers + 1];
    inputs.connected = new boolean[followers + 1];
    inputs.supplyCurrentAmps = new double[followers + 1];
    inputs.appliedVolts = new double[followers + 1];
    inputs.positionRot = 0;
    inputs.velocityRotPS = 0;
  }

  public void periodic() {
    this.updateInputs(inputs);
    Logger.processInputs("Motors/" + name, inputs);
    if (pidTuning) {
      LoggedTunableNumber.ifChanged(this, this::applyAllTuningChanges, tunableNumbers);
    }
    for (int i = 0; i < followers + 1; i++) {
      connectionAlerts[i].set(!inputs.connected[i]);
    }
  }

  private void applyAllTuningChanges(double[] values) {
    applyTuningChange(values);
    for (LoggedTalonFX tuningFollower : tuningFollowers) {
      tuningFollower.applyTuningChange(values);
    }
  }

  private void applyTuningChange(double[] values) {
    if (tunedConfigs != null) {
      tunedConfigs.kP = values[0];
      tunedConfigs.kI = values[1];
      tunedConfigs.kD = values[2];
      tunedConfigs.kG = values[3];
      tunedConfigs.kS = values[4];
      tunedConfigs.kV = values[5];
      tunedConfigs.kA = values[6];
      quickApplyConfig(tunedConfigs);
    }
    if (mmTunedConfigs != null) {
      mmTunedConfigs.MotionMagicCruiseVelocity = values[7];
      mmTunedConfigs.MotionMagicAcceleration = values[8];
      mmTunedConfigs.MotionMagicJerk = values[9];
      quickApplyConfig(mmTunedConfigs);
    }
  }

  public LoggedTalonFX withPIDTunable(SlotConfigs defaultValues, LoggedTalonFX... followers) {
    if (!Constants.tuningMode) return this;
    if (pidTuning) {
      DriverStation.reportWarning("Attempted to initiate PID tuning twice", true);
    }
    pidTuning = true;

    this.tunableNumbers =
        new LoggedTunableNumber[] {
          new LoggedTunableNumber(name + "/kP", defaultValues.kP),
          new LoggedTunableNumber(name + "/kI", defaultValues.kI),
          new LoggedTunableNumber(name + "/kD", defaultValues.kD),
          new LoggedTunableNumber(name + "/kG", defaultValues.kG),
          new LoggedTunableNumber(name + "/kS", defaultValues.kS),
          new LoggedTunableNumber(name + "/kV", defaultValues.kV),
          new LoggedTunableNumber(name + "/kA", defaultValues.kA),
        };

    tunedConfigs = defaultValues;
    tuningFollowers = followers;

    return this;
  }

  public LoggedTalonFX withMMPIDTuning(
      SlotConfigs slotConfig, MotionMagicConfigs mmConfig, LoggedTalonFX... followers) {
    withPIDTunable(slotConfig, followers);
    if (!Constants.tuningMode) return this;
    this.mmTunedConfigs = mmConfig;
    if (mmTuning) {
      DriverStation.reportWarning("Attempted to initiate Motion Magic tuning twice", true);
    }
    mmTuning = true;

    final LoggedTunableNumber[] mmTunableNumbers =
        new LoggedTunableNumber[] {
          new LoggedTunableNumber(name + "/MM Cruise Velocity", mmConfig.MotionMagicCruiseVelocity),
          new LoggedTunableNumber(name + "/MM Acceleration", mmConfig.MotionMagicAcceleration),
          new LoggedTunableNumber(name + "/MM Jerk", mmConfig.MotionMagicJerk),
        };
    final LoggedTunableNumber[] copy =
        new LoggedTunableNumber[this.tunableNumbers.length + mmTunableNumbers.length];
    System.arraycopy(this.tunableNumbers, 0, copy, 0, this.tunableNumbers.length);
    System.arraycopy(
        mmTunableNumbers, 0, copy, this.tunableNumbers.length, mmTunableNumbers.length);
    this.tunableNumbers = copy;
    return this;
  }

  public boolean atSetpoint(Angle setpoint, Angle tolerance) {
    return MathUtil.isNear(
        setpoint.baseUnitMagnitude(),
        getPosition().baseUnitMagnitude(),
        tolerance.baseUnitMagnitude());
  }

  public boolean atSetpoint(AngularVelocity setpoint, AngularVelocity tolerance) {
    return MathUtil.isNear(
        setpoint.baseUnitMagnitude(),
        getVelocity().baseUnitMagnitude(),
        tolerance.baseUnitMagnitude());
  }

  /**
   * Build a standard config with a few commonly adjusted values
   *
   * @param statorCurrentLimit Stator current limit in amps This limit directly affects max torque
   * @param supplyCurrentLimit Supply current limit in amps This limit affects brownouts and battery
   *     usage
   * @param invert Motor direction. Default: CCW positive
   * @param neutralMode Neutral mode. Default: Coast
   * @return the built config
   */
  public static TalonFXConfiguration buildStandardConfig(
      double statorCurrentLimit,
      double supplyCurrentLimit,
      InvertedValue invert,
      NeutralModeValue neutralMode) {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = statorCurrentLimit;
    config.CurrentLimits.SupplyCurrentLimit = supplyCurrentLimit;
    config.MotorOutput.Inverted = invert;
    config.MotorOutput.NeutralMode = neutralMode;
    return config;
  }

  /**
   * Build a standard config with a few commonly adjusted values The NeutralMode is Coast
   *
   * @param statorCurrentLimit Stator current limit in amps This limit directly affects max torque
   * @param supplyCurrentLimit Supply current limit in amps This limit affects brownouts and battery
   *     usage
   * @param invert Motor direction. Default: CCW positive
   * @return the built config
   */
  public static TalonFXConfiguration buildStandardConfig(
      double statorCurrentLimit, double supplyCurrentLimit, InvertedValue invert) {
    return LoggedTalonFX.buildStandardConfig(
        statorCurrentLimit, supplyCurrentLimit, invert, NeutralModeValue.Coast);
  }

  /**
   * Build a standard config with a few commonly adjusted values The InvertValue is CCW positive
   *
   * @param statorCurrentLimit Stator current limit in amps This limit directly affects max torque
   * @param supplyCurrentLimit Supply current limit in amps This limit affects brownouts and battery
   *     usage
   * @param neutralMode Neutral mode. Default: Coast
   * @return the built config
   */
  public static TalonFXConfiguration buildStandardConfig(
      double statorCurrentLimit, double supplyCurrentLimit, NeutralModeValue neutralMode) {
    return LoggedTalonFX.buildStandardConfig(
        statorCurrentLimit,
        supplyCurrentLimit,
        InvertedValue.CounterClockwise_Positive,
        neutralMode);
  }

  public static TalonFXConfiguration buildStandardConfig(
      double statorCurrentLimit, double supplyCurrentLimit) {
    return LoggedTalonFX.buildStandardConfig(
        statorCurrentLimit,
        supplyCurrentLimit,
        InvertedValue.CounterClockwise_Positive,
        NeutralModeValue.Coast);
  }

  public abstract void setControl(ControlRequest controlRequest);

  protected abstract void updateInputs(TalonInputs inputs);

  /**
   * Apply a config until it succeeds.
   *
   * @param config the config to apply
   * @return this for daisy-chaining
   */
  public abstract LoggedTalonFX withConfig(TalonFXConfiguration config);

  /**
   * Apply a config for simulation. This is ignored by a real IO interface. This should be called
   * after a normal config is applied.
   *
   * @param config Function to generate a config. This will only be called in simulation. Takes in
   *     the current config, modifies it for simulation, and returns it. This is meant to make the
   *     simulation resemble reality as much as possible.
   * @return This for daisy-chaining
   */
  public abstract LoggedTalonFX withSimConfig(
      Function<TalonFXConfiguration, TalonFXConfiguration> config);

  public abstract void quickApplyConfig(TalonFXConfiguration config);

  public abstract void quickApplyConfig(SlotConfigs config);

  public abstract void quickApplyConfig(MotionMagicConfigs config);

  public Voltage getPrimaryAppliedVoltage() {
    return getAppliedVoltage(0);
  }

  public Voltage getAppliedVoltage(int follower) {
    return Volt.of(getAppliedVoltageVolts(follower));
  }

  public double getPrimaryAppliedVoltageVolts() {
    return getAppliedVoltageVolts(0);
  }

  public double getAppliedVoltageVolts(int follower) {
    return this.inputs.appliedVolts[follower];
  }

  public Temperature getPrimaryTemperature() {
    return getTemperature(0);
  }

  public Temperature getTemperature(int follower) {
    return Celsius.of(this.inputs.temperatureC[follower]);
  }

  public double getPrimaryTemperatureC() {
    return getTemperatureC(0);
  }

  public double getTemperatureC(int follower) {
    return this.inputs.temperatureC[follower];
  }

  public Current getPrimaryTorqueCurrent() {
    return getTorqueCurrent(0);
  }

  public Current getTorqueCurrent(int follower) {
    return Amp.of(getTorqueCurrentAmps(follower));
  }

  public double getPrimaryTorqueCurrentAmps() {
    return getTorqueCurrentAmps(0);
  }

  public double getTorqueCurrentAmps(int follower) {
    return this.inputs.torqueCurrentAmps[follower];
  }

  public Current getPrimarySupplyCurrent() {
    return getSupplyCurrent(0);
  }

  public Current getSupplyCurrent(int follower) {
    return Amp.of(getSupplyCurrentAmps(follower));
  }

  public double getPrimarySupplyCurrentAmps() {
    return getSupplyCurrentAmps(0);
  }

  public double getSupplyCurrentAmps(int follower) {
    return this.inputs.supplyCurrentAmps[follower];
  }

  public AngularVelocity getVelocity() {
    return velocity.mut_replace(this.inputs.velocityRotPS, RotationsPerSecond);
  }

  public Angle getPosition() {
    return position.mut_replace(this.inputs.positionRot, Rotation);
  }

  public abstract void setPosition(Angle position);
}

/*
LoggedTalon is an abstract clas
RealTalonFX implements LoggedTalon
HardwareTalonFX extends RealTalonFX
SimTalonFX extends RealTalonFX
 */
