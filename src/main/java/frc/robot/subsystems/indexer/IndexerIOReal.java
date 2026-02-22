package frc.robot.subsystems.indexer;


import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.util.SparkUtil;

public class IndexerIOReal implements IndexerIO {

  private final SparkMax motor;
  private IdleMode currentIdleMode = IdleMode.kCoast;

  public IndexerIOReal() {

    motor = new SparkMax(IndexerConstants.MOTOR_ID, MotorType.kBrushless);

    var config = new SparkMaxConfig();

    config
        .smartCurrentLimit(IndexerConstants.CURRENT_LIMIT_AMPS)
        .inverted(IndexerConstants.INVERTED)
        .idleMode(IdleMode.kCoast)
        .voltageCompensation(12.0);

    config
        .encoder
        .positionConversionFactor(2.0 * Math.PI)
        .velocityConversionFactor(2.0 * Math.PI / 60.0);

    SparkUtil.tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {

    inputs.connected = !motor.getFaults().can;

    inputs.velocityRadsPerSec = motor.getEncoder().getVelocity();

    inputs.appliedVoltage = motor.getAppliedOutput() * motor.getBusVoltage();

    inputs.supplyCurrentAmps = motor.getOutputCurrent();

    inputs.statorCurrentAmps = Double.NaN;

    inputs.tempCelsius = motor.getMotorTemperature();
  }

  @Override
  public void applyOutputs(IndexerIOOutputs outputs) {

    switch (outputs.mode) {
      case COAST -> {
        setIdleMode(IdleMode.kCoast);
        motor.set(0.0);
      }

      case VOLTAGE_CONTROL -> {
        setIdleMode(outputs.brakeModeEnabled ? IdleMode.kBrake : IdleMode.kCoast);

        motor.setVoltage(outputs.appliedVoltage);
      }
    }
  }

  private void setIdleMode(IdleMode mode) {

    if (mode == currentIdleMode) return;

    currentIdleMode = mode;

    var config = new SparkMaxConfig();

    config.idleMode(mode);

    motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }
}
