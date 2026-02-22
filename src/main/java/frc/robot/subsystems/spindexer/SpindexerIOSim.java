package frc.robot.subsystems.spindexer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class SpindexerIOSim implements SpindexerIO {
  private FlywheelSim sim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), .004, 1.0), DCMotor.getNEO(1));
  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(SpindexerIOInputs inputs) {
    sim.update(0.020); // Move physics forward 20ms

    inputs.velocityRadsPerSec = sim.getAngularVelocityRPM() / 60.0 * 2 * Math.PI;
    inputs.appliedVoltage = appliedVolts;
  }

  @Override
  public void applyOutputs(SpindexerIOOutputs outputs) {
    if (outputs.mode == SpindexerIOMode.VOLTAGE_CONTROL) {
      appliedVolts = MathUtil.clamp(outputs.appliedVoltage, -12.0, 12.0);
    } else {
      appliedVolts = 0.0;
    }
  }
}
