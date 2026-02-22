package frc.robot.subsystems.intake.rollers;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeRollersIO {

  @AutoLog
  public static class IntakeRollersIOInputs {
    public boolean connected;
    public double velocityRadsPerSec;
    public double appliedVoltage;
    public double supplyCurrentAmps;
    public double statorCurrentAmps;
    public double tempCelsius;
  }

  public enum IntakeRollersIOMode {
    COAST,
    VOLTAGE_CONTROL
  }

  public static class IntakeRollersIOOutputs {
    public IntakeRollersIOMode mode = IntakeRollersIOMode.COAST;

    // Voltage control
    public double appliedVoltage;
    public double velocityRadsPerSec;

    public boolean brakeModeEnabled = true;
  }

  public default void updateInputs(IntakeRollersIOInputs inputs) {}

  default void applyOutputs(IntakeRollersIOOutputs outputs) {}
}
