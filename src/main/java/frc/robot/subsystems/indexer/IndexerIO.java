package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {

  @AutoLog
  public static class IndexerIOInputs {
    public boolean connected;
    public double velocityRadsPerSec;
    public double appliedVoltage;
    public double supplyCurrentAmps;
    public double statorCurrentAmps;
    public double tempCelsius;
  }

  public enum IndexerIOMode {
    COAST,
    VOLTAGE_CONTROL
  }

  public static class IndexerIOOutputs {
    public IndexerIOMode mode = IndexerIOMode.COAST;

    // Voltage control
    public double appliedVoltage;
    public double velocityRadsPerSec;
    public boolean brakeModeEnabled = true;
  }

  public default void updateInputs(IndexerIOInputs inputs) {}

  default void applyOutputs(IndexerIOOutputs outputs) {}
}
