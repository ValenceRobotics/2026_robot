package frc.robot.subsystems.spindexer;

import org.littletonrobotics.junction.AutoLog;

public interface SpindexerIO {

  @AutoLog
  public static class SpindexerIOInputs {
    public boolean connected;
    public double velocityRadsPerSec;
    public double appliedVoltage;
    public double supplyCurrentAmps;
    public double statorCurrentAmps;
    public double tempCelsius;
  }

  public enum SpindexerIOMode {
    COAST,
    VOLTAGE_CONTROL
  }

  public static class SpindexerIOOutputs {
    public SpindexerIOMode mode = SpindexerIOMode.COAST;

    // Voltage control
    public double appliedVoltage;
    public double velocityRadsPerSec;
    public boolean brakeModeEnabled = true;
  }

  public default void updateInputs(SpindexerIOInputs inputs) {}

  default void applyOutputs(SpindexerIOOutputs outputs) {}
}
