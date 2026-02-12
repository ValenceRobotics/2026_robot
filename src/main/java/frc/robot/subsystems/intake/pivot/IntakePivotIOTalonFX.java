package frc.robot.subsystems.intake.pivot;

import static frc.robot.subsystems.intake.IntakeConstants.*;

public class IntakePivotIOTalonFX implements IntakePivotIO {
  // private final TalonFX pivotMotor;
  // private final MotionMagicVoltage motionMagicRequest;

  // public IntakePivotIOTalonFX() {
  //   pivotMotor = new TalonFX(MOTOR_ID);
  //   motionMagicRequest = new MotionMagicVoltage(0).withSlot(0);

  //   configureMotor();
  // }

  // private void configureMotor() {
  //   TalonFXConfiguration config = new TalonFXConfiguration();

  //   // PID Configuration (Slot 0)
  //   Slot0Configs slot0 = config.Slot0;
  //   slot0.kP = kP;
  //   slot0.kI = 0.0;
  //   slot0.kD = kD;
  //   slot0.kV = kF; // Velocity feedforward
  //   slot0.kG = kG; // Gravity feedforward
  //   slot0.GravityType = GravityTypeValue.Arm_Cosine; // For arm mechanisms

  //   // Motion Magic Configuration
  //   MotionMagicConfigs motionMagic = config.MotionMagic;
  //   motionMagic.MotionMagicCruiseVelocity = MAX_VELOCITY;
  //   motionMagic.MotionMagicAcceleration = MAX_ACCEL;
  //   motionMagic.MotionMagicJerk = MAX_ACCEL * 10; // Jerk for S-curve

  //   // Motor configuration
  //   config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
  //   config.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT;
  //   config.CurrentLimits.SupplyCurrentLimitEnable = true;

  //   // Soft limits
  //   config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
  //   config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = LIMIT_FWD;
  //   config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
  //   config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = LIMIT_REV;

  //   // Apply configuration
  //   pivotMotor.getConfigurator().apply(config);
  // }

  // @Override
  // public void updateInputs(IntakePivotIOInputs inputs) {
  //   // Motor values (rotor rotations - before gearing)
  //   inputs.positionRot = pivotMotor.getRotorPosition().getValueAsDouble();
  //   inputs.velocityRotPerSec = pivotMotor.getRotorVelocity().getValueAsDouble();

  //   // Mechanism values (after gearing - this is what we control)
  //   inputs.positionRot = pivotMotor.getPosition().getValueAsDouble();
  //   inputs.velocityRotPerSec = pivotMotor.getVelocity().getValueAsDouble();

  //   inputs.appliedVolts = pivotMotor.getMotorVoltage().getValueAsDouble();
  //   inputs.currentAmps = pivotMotor.getSupplyCurrent().getValueAsDouble();
  //   inputs.tempCelsius = pivotMotor.getDeviceTemp().getValueAsDouble();
  // }

  // @Override
  // public void setPosition(double positionRot) {
  //   pivotMotor.setControl(motionMagicRequest.withPosition(positionRot));
  // }

  // @Override
  // public void setVoltage(double volts) {
  //   pivotMotor.setVoltage(volts);
  // }

  // @Override
  // public void stop() {
  //   pivotMotor.stopMotor();
  // }

  // @Override
  // public void zeroToCurrentPos() {
  //   pivotMotor.setPosition(0);
  // }

  // @Override
  // public void setCurrentPosition(double positionRot) {
  //   pivotMotor.setPosition(positionRot);
  // }
}
