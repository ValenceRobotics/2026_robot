package frc.robot.subsystems.intake.rollers;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeRollersIO {

  @AutoLog
  public static class IntakeRollersIOInputs {
    public double velocityRPM = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double tempCelsius = 0.0;
  }

  public enum IntakeRollersIOMode {
    BRAKE,
    COAST,
    VOLTAGE_CONTROL
  }

  public static class IntakeRollersIOOutputs {
    public IntakeRollersIOMode mode = IntakeRollersIOMode.BRAKE;

    // Voltage control
    public double appliedVoltage = 0.0;

    // TODO: Implement Closed loop control (maybe not necessary)
    // public double velocity = 0.0;
    // public double kP = 0.0;
    // public double kD = 0.0;
    // public double feedforward = 0.0;

    public boolean brakeModeEnabled = true;
  }

  public default void updateInputs(IntakeRollersIOInputs inputs) {}

  default void applyOutputs(IntakeRollersIOOutputs outputs) {}
}
