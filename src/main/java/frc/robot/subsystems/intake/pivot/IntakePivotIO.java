package frc.robot.subsystems.intake.pivot;

import org.littletonrobotics.junction.AutoLog;

public interface IntakePivotIO {
  @AutoLog
  public static class IntakePivotIOInputs {
    public double positionRot = 0.0;
    public double velocityRotPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double tempCelsius = 0.0;

    public boolean forwardLimitSwitch = false;
    public boolean reverseLimitSwitch = false;
  }

  public default void updateInputs(IntakePivotIOInputs inputs) {}

  public default void setPosition(double positionRot) {}

  public default void setVoltage(double volts) {}

  public default void stop() {}

  public default void zeroToCurrentPos() {}

  public default void setCurrentPosition(double positionRot) {}
}
