package frc.robot.subsystems.shooter.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {

  @AutoLog
  public static class HoodIOInputs {
    // TODO: add either absolute encoder or limit switch
    public boolean motorConnected = false;
    public double positionRads = 0.0;
    // public double motorRotations = 0.0; from last yrs elevator code
    // public double encoderRotations = 0.0;
    public double velocityRadsPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
  }

  // modes for hood control
  public static enum HoodIOOutputMode {
    BRAKE,
    COAST,
    CLOSED_LOOP
  }

  public static class HoodIOOutputs {

    public HoodIOOutputMode mode = HoodIOOutputMode.BRAKE;
    // Closed loop control
    public double positionRad = 0.0;
    public double velocityRadsPerSec = 0.0;
    public double kP = 0.0;
    public double kD = 0.0;
  }

  public default void updateInputs(HoodIOInputs inputs) {}

  public default void applyOutputs(HoodIOOutputs outputs) {}
}
