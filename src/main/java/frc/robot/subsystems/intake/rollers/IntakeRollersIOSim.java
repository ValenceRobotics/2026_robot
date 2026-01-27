package frc.robot.subsystems.intake.rollers;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class IntakeRollersIOSim implements IntakeRollersIO {
  private final FlywheelSim sim;
  private double appliedVolts = 0.0;
  private boolean simulatedGamePiece = false;

  public IntakeRollersIOSim() {
    sim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getFalcon500(1), .004, 1.0),
            DCMotor.getFalcon500(1),
            null);
  }

  @Override
  public void updateInputs(IntakeRollersIOInputs inputs) {
    sim.update(0.02);

    inputs.velocityRPM = sim.getAngularVelocityRPM();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = sim.getCurrentDrawAmps();
    inputs.tempCelsius = 25.0;

    // Simulate game piece detection when spinning fast enough
    if (Math.abs(inputs.velocityRPM) > 100) {
      simulatedGamePiece = true;
    }
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = volts;
    sim.setInputVoltage(volts);
  }

  @Override
  public void setSpeed(double speed) {
    setVoltage(speed * 12.0);
  }

  @Override
  public void stop() {
    setVoltage(0);
  }

  // Helper for testing
  public void setSimulatedGamePiece(boolean hasPiece) {
    simulatedGamePiece = hasPiece;
  }
}
