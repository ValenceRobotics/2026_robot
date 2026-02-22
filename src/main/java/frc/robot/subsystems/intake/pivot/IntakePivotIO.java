package frc.robot.subsystems.intake.pivot;

import org.littletonrobotics.junction.AutoLog;

public interface IntakePivotIO {
  @AutoLog
  public static class IntakePivotIOInputs {
    public boolean connected;
    public double positionRad;
    public double velocityRadsPerSec;
    public double appliedVolts;
    public double currentAmps;
    public double tempCelsius;
    public double appliedVoltage;
    public double supplyCurrentAmps;
    public double statorCurrentAmps;

    // limit switch inputs; not done yet
    public boolean forwardLimitSwitch = false;
    public boolean reverseLimitSwitch = false;
  }

  public static enum IntakePivotIOOutputMode {
    BRAKE,
    COAST,
    CLOSED_LOOP
  }

  public static class IntakePivotIOOutputs {
    public IntakePivotIOOutputMode mode = IntakePivotIOOutputMode.BRAKE;
    // Closed loop control
    public double positionRad = 0.0;
    public double velocityRadsPerSec = 0.0;
    public double kP = 0.0;
    public double kD = 0.0;
  }

  public default void updateInputs(IntakePivotIOInputs inputs) {}

  public default void applyOutputs(IntakePivotIOOutputs outputs) {}

  // public default void setPosition(double positionRot) {}

  // public default void setVoltage(double volts) {}

  public default void stop() {}

  public default void zeroToCurrentPos() {}

  public default void setCurrentPosition(double positionRad) {}
}
