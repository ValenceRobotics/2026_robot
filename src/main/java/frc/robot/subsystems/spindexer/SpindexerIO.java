package frc.robot.subsystems.spindexer;

import org.littletonrobotics.junction.AutoLog;

public interface SpindexerIO {
  @AutoLog
  public static class SpindexerIOInputs {
    public double positionRotations = 0.0;
    public double velocityRps = 0.0;
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[] {};
  }

  public default void updateInputs(SpindexerIOInputs inputs) {}

  public default void setVoltage(double volts) {}

  public default void setBrakeMode(boolean enable) {}
}
