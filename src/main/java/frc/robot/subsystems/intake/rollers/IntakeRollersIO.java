package frc.robot.subsystems.intake.rollers;

public interface IntakeRollersIO {
  public static class IntakeRollersIOInputs {
    public double velocityRPM = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double tempCelsius = 0.0;
  }

  public default void updateInputs(IntakeRollersIOInputs inputs) {}

  public default void setVoltage(double volts) {}

  public default void setSpeed(double speed) {}

  public default void stop() {}
}
