package frc.robot.subsystems.shooter.flywheel;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.util.PhoenixUtil;

public class FlywheelIOTalonFX implements FlywheelIO {

  private final TalonFX leader;
  private final TalonFX follower;

  // Status signals
  private final StatusSignal<AngularVelocity> leaderVelocity;
  private final StatusSignal<Voltage> leaderVoltage;
  private final StatusSignal<Current> leaderSupplyCurrent;
  private final StatusSignal<Current> leaderStatorCurrent;
  private final StatusSignal<Temperature> leaderTemp;
  private final StatusSignal<Temperature> followerTemp;

  // Reuse control requests — never allocate in the loop
  private final VelocityTorqueCurrentFOC velocityRequest = new VelocityTorqueCurrentFOC(0.0);
  private final CoastOut coastRequest = new CoastOut();

  public FlywheelIOTalonFX() {
    leader = new TalonFX(ShooterConstants.FlywheelConstants.leaderMotorId);
    follower = new TalonFX(ShooterConstants.FlywheelConstants.followerMotorId);

    var config = new TalonFXConfiguration();

    // 1.66:1 reduction — sensor reads motor, mechanism is flywheel
    config.Feedback.SensorToMechanismRatio = ShooterConstants.FlywheelConstants.GEAR_RATIO;

    config.TorqueCurrent.PeakForwardTorqueCurrent = 120.0;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -120.0;

    config.CurrentLimits.StatorCurrentLimit = 120.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast; // flywheel should coast when idle

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

    // Cache signals
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

    var followerTempStatus = BaseStatusSignal.refreshAll(followerTemp);
    BaseStatusSignal.refreshAll(leaderTemp);
    inputs.followerConnected = followerTempStatus.isOK();

    inputs.velocityRadsPerSec = leaderVelocity.getValueAsDouble() * 2.0 * Math.PI;
    inputs.appliedVoltage = leaderVoltage.getValueAsDouble();
    inputs.supplyCurrentAmps = leaderSupplyCurrent.getValueAsDouble();
    inputs.statorCurrentAmps = leaderStatorCurrent.getValueAsDouble();
    inputs.tempCelsius = leaderTemp.getValueAsDouble();
    inputs.followerTempCelsius = followerTemp.getValueAsDouble();
  }

  @Override
  public void applyOutputs(FlywheelIOOutputs outputs) {
    switch (outputs.mode) {
      case COAST -> leader.setControl(coastRequest);
      case VELOCITY -> {
        double velocityRotPerSec = outputs.velocityRadsPerSec / (2.0 * Math.PI);
        leader.setControl(velocityRequest.withVelocity(velocityRotPerSec));
      }
    }
  }
}
