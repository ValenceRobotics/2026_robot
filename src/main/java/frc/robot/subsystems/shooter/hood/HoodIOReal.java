package frc.robot.subsystems.shooter.hood;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.util.SparkUtil;
import org.littletonrobotics.junction.Logger;

public class HoodIOReal implements HoodIO {

  private SparkMax hood =
      new SparkMax(ShooterConstants.HoodConstants.hoodMotorId, MotorType.kBrushless);
  private SparkClosedLoopController hoodController;
  private IdleMode currentIdleMode = IdleMode.kBrake;

  public HoodIOReal() {
    hoodController = hood.getClosedLoopController();

    var hoodConfig = new SparkMaxConfig();

    hoodConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(ShooterConstants.HoodConstants.currentLimit)
        .voltageCompensation(12.0);

    hoodConfig
        .encoder
        .positionConversionFactor((1.0 / ShooterConstants.HoodConstants.gearRatio) * 2 * Math.PI)
        .velocityConversionFactor(
            (1.0 / ShooterConstants.HoodConstants.gearRatio) * 2 * Math.PI / 60.0);

    hoodConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(
            ShooterConstants.HoodConstants.kPReal.get(),
            0.0,
            ShooterConstants.HoodConstants.kDReal.get())
        .outputRange(-1, 1);

    // These should be in rad/s and rad/s² after conversion
    hoodConfig
        .closedLoop
        .maxMotion
        .cruiseVelocity(ShooterConstants.HoodConstants.cruiseVelocity)
        .maxAcceleration(ShooterConstants.HoodConstants.maxAcceleration)
        .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);

    SparkUtil.tryUntilOk(
        hood,
        5,
        () ->
            hood.configure(
                hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.motorConnected = !hood.getFaults().can;
    inputs.positionRads = hood.getEncoder().getPosition();
    inputs.velocityRadsPerSec = hood.getEncoder().getVelocity();
    inputs.appliedVolts = hood.getAppliedOutput() * hood.getBusVoltage();
    inputs.supplyCurrentAmps = hood.getOutputCurrent();
    inputs.tempCelsius = hood.getMotorTemperature();
  }

  @Override
  public void applyOutputs(HoodIOOutputs outputs) {

    // push new config when tunable numbers change
    if (ShooterConstants.HoodConstants.kPReal.hasChanged(hashCode())
        || ShooterConstants.HoodConstants.kDReal.hasChanged(hashCode())) {
      var updateConfig = new SparkMaxConfig();
      updateConfig.closedLoop.pid(
          ShooterConstants.HoodConstants.kPReal.get(),
          0.0,
          ShooterConstants.HoodConstants.kDReal.get());
      hood.configure(
          updateConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    switch (outputs.mode) {
      case BRAKE -> {
        setIdleMode(IdleMode.kBrake);
        hood.set(0.0);
      }
      case COAST -> {
        setIdleMode(IdleMode.kCoast);
        hood.set(0.0);
      }
      case CLOSED_LOOP -> {
        setIdleMode(IdleMode.kBrake);
        double ff = ShooterConstants.HoodConstants.kGReal.get();
        hoodController.setSetpoint(
            outputs.positionRad, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, ff);
        Logger.recordOutput("Hood/Setpoint", outputs.positionRad);
      }
    }
  }

  private void setIdleMode(IdleMode mode) {
    if (currentIdleMode == mode) return;
    currentIdleMode = mode;
    var idleConfig = new SparkMaxConfig();
    idleConfig.idleMode(mode);
    hood.configure(idleConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }
}
