package frc.robot.subsystems.shooter.flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
  @AutoLog
  public static class FlywheelIOInputs {
    public boolean connected;
    public double velocityRadsPerSec;
    public double appliedVoltage;
    public double tempCelsius;
    public double supplyCurrentAmps = 0.0;
    public double statorCurrentAmps = 0.0;
    public boolean followerConnected;
    public double followerTempCelsius;
  }

  public static enum FlywheelIOOutputMode {
    COAST,
    VELOCITY
  }

  public static class FlywheelIOOutputs {
    public FlywheelIOOutputMode mode = FlywheelIOOutputMode.COAST;
    public double velocityRadsPerSec = 0.0;
  }

  default void updateInputs(FlywheelIOInputs inputs) {}

  default void applyOutputs(FlywheelIOOutputs outputs) {}

  public default void setFlywheelOpenLoop(double output) {}
}
