package frc.robot.subsystems.shooter.flywheel;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterConstants.FlywheelConstants;
import frc.robot.util.PhoenixUtil;
import org.littletonrobotics.junction.AutoLogOutput;

public class FlywheelIOTalonFX implements FlywheelIO {

  private final TalonFX leader;
  private final TalonFX follower;

  private final StatusSignal<AngularVelocity> leaderVelocity;
  private final StatusSignal<Voltage> leaderVoltage;
  private final StatusSignal<Current> leaderSupplyCurrent;
  private final StatusSignal<Current> leaderStatorCurrent;
  private final StatusSignal<Temperature> leaderTemp;
  private final StatusSignal<Temperature> followerTemp;

  private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);
  private final CoastOut coastRequest = new CoastOut();
  private final VoltageOut voltageRequest = new VoltageOut(0.0);

  public FlywheelIOTalonFX() {
    leader = new TalonFX(ShooterConstants.FlywheelConstants.leaderMotorId);
    follower = new TalonFX(ShooterConstants.FlywheelConstants.followerMotorId);

    var config = new TalonFXConfiguration();

    config.Feedback.SensorToMechanismRatio = 0;

    config.CurrentLimits.StatorCurrentLimit = 120.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.Slot0.kP = ShooterConstants.FlywheelConstants.kP.get();
    config.Slot0.kI = 0.0;
    config.Slot0.kD = ShooterConstants.FlywheelConstants.kD.get();

    config.Slot0.kS = ShooterConstants.FlywheelConstants.kS.get();
    config.Slot0.kV = ShooterConstants.FlywheelConstants.kV.get();

    PhoenixUtil.tryUntilOk(10, () -> leader.getConfigurator().apply(config));
    PhoenixUtil.tryUntilOk(10, () -> follower.getConfigurator().apply(config));

    PhoenixUtil.tryUntilOk(
        10,
        () -> follower.setControl(new Follower(leader.getDeviceID(), MotorAlignmentValue.Opposed)));

    // cache signals
    leaderVelocity = leader.getVelocity();
    leaderVoltage = leader.getMotorVoltage();
    leaderSupplyCurrent = leader.getSupplyCurrent();
    leaderStatorCurrent = leader.getStatorCurrent();
    leaderTemp = leader.getDeviceTemp();
    followerTemp = follower.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0, leaderVelocity, leaderVoltage, leaderSupplyCurrent, leaderStatorCurrent);
    BaseStatusSignal.setUpdateFrequencyForAll(4.0, leaderTemp, followerTemp);

    leader.optimizeBusUtilization();
    follower.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.connected =
        BaseStatusSignal.refreshAll(
                leaderVelocity, leaderVoltage, leaderSupplyCurrent, leaderStatorCurrent)
            .isOK();

    inputs.followerConnected = BaseStatusSignal.refreshAll(followerTemp).isOK();
    BaseStatusSignal.refreshAll(leaderTemp);

    inputs.velocityRadsPerSec =
        -leaderVelocity.getValueAsDouble() * 2.0 * Math.PI * (1.0 / FlywheelConstants.GEAR_RATIO);

    inputs.appliedVoltage = leaderVoltage.getValueAsDouble();
    inputs.supplyCurrentAmps = leaderSupplyCurrent.getValueAsDouble();
    inputs.statorCurrentAmps = leaderStatorCurrent.getValueAsDouble();
    inputs.tempCelsius = leaderTemp.getValueAsDouble();
    inputs.followerTempCelsius = followerTemp.getValueAsDouble();
  }

  @Override
  public void applyOutputs(FlywheelIOOutputs outputs) {

    if (ShooterConstants.FlywheelConstants.kP.hasChanged(hashCode())
        || ShooterConstants.FlywheelConstants.kD.hasChanged(hashCode())
        || ShooterConstants.FlywheelConstants.kS.hasChanged(hashCode())
        || ShooterConstants.FlywheelConstants.kV.hasChanged(hashCode())
        || ShooterConstants.FlywheelConstants.kG.hasChanged(hashCode())) {
      TalonFXConfiguration updateConfig = new TalonFXConfiguration();
      updateConfig.Slot0.kP = ShooterConstants.FlywheelConstants.kP.get();
      updateConfig.Slot0.kI = 0.0;
      updateConfig.Slot0.kD = ShooterConstants.FlywheelConstants.kD.get();

      updateConfig.Slot0.kS = ShooterConstants.FlywheelConstants.kS.get();
      updateConfig.Slot0.kV = ShooterConstants.FlywheelConstants.kV.get();

      PhoenixUtil.tryUntilOk(10, () -> leader.getConfigurator().apply(updateConfig));
      PhoenixUtil.tryUntilOk(10, () -> follower.getConfigurator().apply(updateConfig));

      PhoenixUtil.tryUntilOk(
          10,
          () ->
              follower.setControl(new Follower(leader.getDeviceID(), MotorAlignmentValue.Opposed)));
    }

    switch (outputs.mode) {
      case COAST -> leader.setControl(coastRequest);

      case VELOCITY -> {
        double velocityRotPerSec = -outputs.velocityRadsPerSec / (2.0 * Math.PI);

        leader.setControl(velocityRequest.withVelocity(velocityRotPerSec));
      }
    }
  }

  @AutoLogOutput(key = "Flywheel/AppliedMotor1Closed")
  public double getMotor1Applied() {
    return leader.getClosedLoopOutput().getValueAsDouble();
  }

  public void setFlywheelOpenLoop(double output) {
    leader.setControl(voltageRequest.withOutput(output));
  }
}
