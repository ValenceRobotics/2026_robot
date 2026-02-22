package frc.robot.subsystems.intake.pivot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.util.PhoenixUtil;

public class IntakePivotIOTalonFX implements IntakePivotIO {

  private final TalonFX leader;

  private final StatusSignal<Angle> leaderPosition;
  private final StatusSignal<AngularVelocity> leaderVelocity;
  private final StatusSignal<Voltage> leaderVoltage;
  private final StatusSignal<Current> leaderSupplyCurrent;
  private final StatusSignal<Current> leaderStatorCurrent;
  private final StatusSignal<Temperature> leaderTemp;

  private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0.0);
  private final StaticBrake brakeRequest = new StaticBrake();
  private final CoastOut coastRequest = new CoastOut();

  public IntakePivotIOTalonFX() {
    leader = new TalonFX(IntakeConstants.PivotConstants.motorId);

    var config = new TalonFXConfiguration();

    config.Feedback.SensorToMechanismRatio = IntakeConstants.PivotConstants.GEAR_RATIO;

    config.TorqueCurrent.PeakForwardTorqueCurrent =
        IntakeConstants.PivotConstants.peakForwardTorqueA;
    config.TorqueCurrent.PeakReverseTorqueCurrent =
        IntakeConstants.PivotConstants.peakReverseTorqueA;

    config.CurrentLimits.StatorCurrentLimit = IntakeConstants.PivotConstants.statorLimitA;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.MotorOutput.Inverted =
        IntakeConstants.PivotConstants.inverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.Slot0.kP = IntakeConstants.PivotConstants.kP.get();
    config.Slot0.kI = 0.0;
    config.Slot0.kD = IntakeConstants.PivotConstants.kD.get();

    config.Slot0.kS = IntakeConstants.PivotConstants.kS.get();
    config.Slot0.kV = IntakeConstants.PivotConstants.kV.get();

    config.MotionMagic.MotionMagicCruiseVelocity =
        IntakeConstants.PivotConstants.mmCruiseVelocityRotPerSec;
    config.MotionMagic.MotionMagicAcceleration = IntakeConstants.PivotConstants.mmAccelRotPerSec2;
    config.MotionMagic.MotionMagicJerk = IntakeConstants.PivotConstants.mmJerkRotPerSec3;

    PhoenixUtil.tryUntilOk(10, () -> leader.getConfigurator().apply(config));

    // Cache signals
    leaderPosition = leader.getPosition();
    leaderVelocity = leader.getVelocity();
    leaderVoltage = leader.getMotorVoltage();
    leaderSupplyCurrent = leader.getSupplyCurrent();
    leaderStatorCurrent = leader.getStatorCurrent();
    leaderTemp = leader.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0,
        leaderPosition,
        leaderVelocity,
        leaderVoltage,
        leaderSupplyCurrent,
        leaderStatorCurrent);
    BaseStatusSignal.setUpdateFrequencyForAll(4.0, leaderTemp);

    leader.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(IntakePivotIOInputs inputs) {
    inputs.connected =
        BaseStatusSignal.refreshAll(
                leaderPosition,
                leaderVelocity,
                leaderVoltage,
                leaderSupplyCurrent,
                leaderStatorCurrent)
            .isOK();

    BaseStatusSignal.refreshAll(leaderTemp);

    inputs.positionRad = leaderPosition.getValueAsDouble() * 2.0 * Math.PI;
    inputs.velocityRadsPerSec = leaderVelocity.getValueAsDouble() * 2.0 * Math.PI;

    inputs.appliedVoltage = leaderVoltage.getValueAsDouble();
    inputs.supplyCurrentAmps = leaderSupplyCurrent.getValueAsDouble();
    inputs.statorCurrentAmps = leaderStatorCurrent.getValueAsDouble();
    inputs.tempCelsius = leaderTemp.getValueAsDouble();
  }

  @Override
  public void applyOutputs(IntakePivotIOOutputs outputs) {
    switch (outputs.mode) {
      case COAST -> leader.setControl(coastRequest);

      case BRAKE -> leader.setControl(brakeRequest);

      case CLOSED_LOOP -> {
        double positionRot = outputs.positionRad / (2.0 * Math.PI);
        leader.setControl(motionMagicRequest.withPosition(positionRot));
      }
    }
  }
}
