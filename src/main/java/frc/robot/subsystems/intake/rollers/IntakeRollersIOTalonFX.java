package frc.robot.subsystems.intake.rollers;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeRollersIOTalonFX implements IntakeRollersIO {
  private final TalonFX motor;
  private final VoltageOut voltageRequest;
  private final DutyCycleOut dutyCycleRequest;

  public IntakeRollersIOTalonFX() {
    motor = new TalonFX(MOTOR_ID);

    voltageRequest = new VoltageOut(0);
    dutyCycleRequest = new DutyCycleOut(0);

    configureMotor();
  }

  private void configureMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    // Coast mode so rollers can be backdriven if needed
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // Current limiting to prevent motor burnout
    config.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    motor.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(IntakeRollersIOInputs inputs) {
    inputs.velocityRPM = motor.getVelocity().getValueAsDouble() * 60.0;
    inputs.appliedVolts = motor.getMotorVoltage().getValueAsDouble();
    inputs.currentAmps = motor.getSupplyCurrent().getValueAsDouble();
    inputs.tempCelsius = motor.getDeviceTemp().getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setSpeed(double speed) {
    motor.setControl(dutyCycleRequest.withOutput(speed));
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }
}
