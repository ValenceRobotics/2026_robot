package frc.robot.subsystems.shooter.hood;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterConstants.HoodConstants;
import frc.robot.util.SparkUtil;

public class HoodIOReal implements HoodIO {

  private SparkMax hood =
      new SparkMax(ShooterConstants.HoodConstants.hoodMotorId, MotorType.kBrushless);
  private SparkClosedLoopController hoodController;
  private IdleMode currentIdleMode = IdleMode.kBrake;
  private final DigitalInput bottomSwitch = new DigitalInput(9); // assuming wired into rio 
  private boolean lastBottomPressed = false;
  // private final SparkLimitSwitch bottomSwitch = hood.getReverseLimitSwitch(); // this is assuming is wired in to sparkmax

  public HoodIOReal() {
    hoodController = hood.getClosedLoopController();

    var hoodConfig = new SparkMaxConfig();

    hoodConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(ShooterConstants.HoodConstants.currentLimit)
        .voltageCompensation(12.0);

    hoodConfig.inverted(true);

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

    SparkUtil.tryUntilOk(
        hood,
        5,
        () ->
            hood.configure(
                hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    hood.getEncoder().setPosition(Units.degreesToRadians(10));
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.motorConnected = !hood.getFaults().can;
    inputs.positionRads = hood.getEncoder().getPosition();
    inputs.velocityRadsPerSec = hood.getEncoder().getVelocity();
    inputs.appliedVolts = hood.getAppliedOutput() * hood.getBusVoltage();
    inputs.supplyCurrentAmps = hood.getOutputCurrent();
    inputs.tempCelsius = hood.getMotorTemperature();

    boolean bottomPressed = isBottomPressed();
    inputs.bottomLimitSwitch = bottomPressed;
    if (bottomPressed && !lastBottomPressed) {
      hood.getEncoder().setPosition(Units.degreesToRadians(10));
    }
    lastBottomPressed = bottomPressed;

  }

  @Override
  public void applyOutputs(HoodIOOutputs outputs) {
    if (ShooterConstants.HoodConstants.kPReal.hasChanged(hashCode())
        || ShooterConstants.HoodConstants.kDReal.hasChanged(hashCode())) {
      SparkMaxConfig updateConfig = new SparkMaxConfig();
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
        double setpoint = outputs.positionRad;
        if (isBottomPressed()) {
          setpoint = Math.max(setpoint, HoodConstants.MIN_ANGLE);
        }

        double error = setpoint - hood.getEncoder().getPosition();
        if (Math.abs(error)
            < Units.degreesToRadians(ShooterConstants.HoodConstants.motorStopToleranceDeg.get())) {
          hood.setVoltage(0);
        } else {
          double ff = ShooterConstants.HoodConstants.kGReal.get();
          hoodController.setSetpoint(setpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0, ff);
        }
      }
      case VOLTAGE_CONTROL -> {
        setIdleMode(IdleMode.kBrake);
        double volts = MathUtil.clamp(outputs.voltage, -12.0, 12.0);
        hood.setVoltage(volts);
      }
    }
  }

  public boolean isBottomPressed() {
    return bottomSwitch.get();
  }
  
  private void setIdleMode(IdleMode mode) {
    if (currentIdleMode == mode) return;
    currentIdleMode = mode;
    var idleConfig = new SparkMaxConfig();
    idleConfig.idleMode(mode);
    hood.configure(idleConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }
}
