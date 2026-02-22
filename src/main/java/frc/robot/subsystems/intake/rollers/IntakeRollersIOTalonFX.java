package frc.robot.subsystems.intake.rollers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.util.PhoenixUtil;

public class IntakeRollersIOTalonFX implements IntakeRollersIO {

  private final TalonFX intakeRoller;

  private final StatusSignal<AngularVelocity> leaderVelocity;
  private final StatusSignal<Voltage> leaderVoltage;
  private final StatusSignal<Current> leaderSupplyCurrent;
  private final StatusSignal<Current> leaderStatorCurrent;
  private final StatusSignal<Temperature> leaderTemp;

  private final CoastOut coastRequest = new CoastOut();
  private final VoltageOut voltageRequest = new VoltageOut(0.0);

  public IntakeRollersIOTalonFX() {
    intakeRoller = new TalonFX(IntakeConstants.RollersConstants.motorId);
    var config = new TalonFXConfiguration();

    config.Feedback.SensorToMechanismRatio = IntakeConstants.RollersConstants.GEAR_RATIO;

    config.TorqueCurrent.PeakForwardTorqueCurrent = 120.0;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -120.0;

    config.CurrentLimits.StatorCurrentLimit = 120.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    PhoenixUtil.tryUntilOk(10, () -> intakeRoller.getConfigurator().apply(config));

    // Cache signals
    leaderVelocity = intakeRoller.getVelocity();
    leaderVoltage = intakeRoller.getMotorVoltage();
    leaderSupplyCurrent = intakeRoller.getSupplyCurrent();
    leaderStatorCurrent = intakeRoller.getStatorCurrent();
    leaderTemp = intakeRoller.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0, leaderVelocity, leaderVoltage, leaderSupplyCurrent, leaderStatorCurrent);
    BaseStatusSignal.setUpdateFrequencyForAll(4.0, leaderTemp);

    intakeRoller.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(IntakeRollersIOInputs inputs) {
    inputs.connected =
        BaseStatusSignal.refreshAll(
                leaderVelocity, leaderVoltage, leaderSupplyCurrent, leaderStatorCurrent)
            .isOK();

    BaseStatusSignal.refreshAll(leaderTemp);

    inputs.velocityRadsPerSec = leaderVelocity.getValueAsDouble() * 2.0 * Math.PI;
    inputs.appliedVoltage = leaderVoltage.getValueAsDouble();
    inputs.supplyCurrentAmps = leaderSupplyCurrent.getValueAsDouble();
    inputs.statorCurrentAmps = leaderStatorCurrent.getValueAsDouble();
    inputs.tempCelsius = leaderTemp.getValueAsDouble();
  }

  @Override
  public void applyOutputs(IntakeRollersIOOutputs outputs) {
    switch (outputs.mode) {
      case COAST -> intakeRoller.setControl(coastRequest);
      case VOLTAGE_CONTROL -> {
        double volts = MathUtil.clamp(outputs.appliedVoltage, -12.0, 12.0);
        intakeRoller.setControl(voltageRequest.withOutput(volts));
      }
    }
  }
}
