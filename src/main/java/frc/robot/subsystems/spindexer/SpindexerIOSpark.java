package frc.robot.subsystems.spindexer;

import static frc.robot.subsystems.spindexer.SpindexerConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

public class SpindexerIOSpark implements SpindexerIO {
  private final SparkMax motor;
  private final SparkMaxConfig config;

  public SpindexerIOSpark() {

    motor = new SparkMax(MOTOR_ID, MotorType.kBrushless);
    config = new SparkMaxConfig();

    config.smartCurrentLimit(30);
    config.inverted(false);
    config.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake);

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(SpindexerIOInputs inputs) {
    inputs.positionRotations = motor.getEncoder().getPosition();
    inputs.velocityRps = motor.getEncoder().getVelocity() / 60.0;
    inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.currentAmps = new double[] {motor.getOutputCurrent()};
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }
}
