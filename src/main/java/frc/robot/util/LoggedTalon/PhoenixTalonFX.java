package frc.robot.util.LoggedTalon;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.*;
import frc.robot.util.LoggedTalon.Follower.PhoenixTalonFollower;
import frc.robot.util.PhoenixUtil;
import java.util.function.Function;

public class PhoenixTalonFX extends LoggedTalonFX {
  protected final TalonFX[] talonFX;
  private final Debouncer[] connectionDebouncer;

  private final StatusSignal<Voltage>[] voltageSignal;
  private final StatusSignal<Current>[] torqueCurrentSignal;
  private final StatusSignal<Current>[] supplyCurrentSignal;
  private final StatusSignal<Temperature>[] temperatureSignal;
  private final StatusSignal<AngularVelocity> velocitySignal;
  private final StatusSignal<Angle> positionSignal;

  @SuppressWarnings("unchecked")
  public PhoenixTalonFX(int canID, CANBus canBus, String name, PhoenixTalonFollower... followers) {

    super(name, followers.length);

    talonFX = new TalonFX[followers.length + 1];
    connectionDebouncer = new Debouncer[followers.length + 1];
    // The trust me bro guarantee
    voltageSignal = (StatusSignal<Voltage>[]) new StatusSignal[followers.length + 1];
    torqueCurrentSignal = (StatusSignal<Current>[]) new StatusSignal[followers.length + 1];
    supplyCurrentSignal = (StatusSignal<Current>[]) new StatusSignal[followers.length + 1];
    temperatureSignal = (StatusSignal<Temperature>[]) new StatusSignal[followers.length + 1];

    Follower follower = new Follower(canID, MotorAlignmentValue.Aligned);
    for (int i = 0; i <= followers.length; i++) {
      if (i == 0) {
        talonFX[0] = new TalonFX(canID, canBus);
      } else {
        talonFX[i] = new TalonFX(followers[i - 1].canid(), canBus);
        talonFX[i].setControl(follower.withMotorAlignment(followers[i - 1].opposeDirection()));
      }
      connectionDebouncer[i] = new Debouncer(0.5);
      voltageSignal[i] = talonFX[i].getMotorVoltage();
      torqueCurrentSignal[i] = talonFX[i].getTorqueCurrent();
      supplyCurrentSignal[i] = talonFX[i].getSupplyCurrent();
      temperatureSignal[i] = talonFX[i].getDeviceTemp();
      BaseStatusSignal.setUpdateFrequencyForAll(PhoenixUtil.kRioSignalUpdateFrequency,voltageSignal[i], torqueCurrentSignal[i], supplyCurrentSignal[i], temperatureSignal[i]);
      talonFX[i].optimizeBusUtilization(PhoenixUtil.kOptimizedSignalFrequency);


    }
    velocitySignal = talonFX[0].getVelocity();
    positionSignal = talonFX[0].getPosition();

    PhoenixUtil.registerSignals(canBus,voltageSignal);
    PhoenixUtil.registerSignals(canBus,torqueCurrentSignal);
    PhoenixUtil.registerSignals(canBus,supplyCurrentSignal);
    PhoenixUtil.registerSignals(canBus,velocitySignal,positionSignal);


  }

  @Override
  public void setControl(ControlRequest controlRequest) {
    talonFX[0].setControl(controlRequest);
  }

  @Override
  protected void updateInputs(TalonFXInputs inputs) {
    for (int i = 0; i <= super.followers; i++) {
      inputs.connected[i] = connectionDebouncer[i].calculate(
          BaseStatusSignal.isAllGood(voltageSignal[i],torqueCurrentSignal[i],supplyCurrentSignal[i],temperatureSignal[i])
          && (i != 0 || BaseStatusSignal.isAllGood(positionSignal, velocitySignal))
      );
      inputs.appliedVolts[i] = voltageSignal[i].getValueAsDouble();
      inputs.torqueCurrentAmps[i] = torqueCurrentSignal[i].getValueAsDouble();
      inputs.supplyCurrentAmps[i] = supplyCurrentSignal[i].getValueAsDouble();
      inputs.temperatureC[i] = temperatureSignal[i].getValueAsDouble();
    }
    inputs.positionRot = positionSignal.getValueAsDouble();
    inputs.velocityRotPS = velocitySignal.getValueAsDouble();
  }

  @Override
  public LoggedTalonFX withConfig(TalonFXConfiguration config) {
    PhoenixUtil.tryUntilOk(5, () -> talonFX[0].getConfigurator().apply(config));
    return this;
  }

  @Override
  public LoggedTalonFX withSimConfig(Function<TalonFXConfiguration, TalonFXConfiguration> config) {
    return this;
  }

  @Override
  public void quickApplyConfig(TalonFXConfiguration config) {
    PhoenixUtil.tryUntilOk(3, () -> talonFX[0].getConfigurator().apply(config));
  }

  @Override
  public void setPosition(Angle position) {
    talonFX[0].setPosition(position);
  }
}
