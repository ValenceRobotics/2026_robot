package frc.robot.subsystems.intake.rollers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class IntakeRollersIOSim implements IntakeRollersIO {

  private final FlywheelSim sim;
  private double appliedVolts = 0.0;

  public IntakeRollersIOSim() {
    sim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getFalcon500(1), 0.004, 1.0),
            DCMotor.getFalcon500(1));
  }

  @Override
  public void updateInputs(IntakeRollersIOInputs inputs) {

    sim.update(0.02);

    inputs.velocityRPM = sim.getAngularVelocityRPM();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = sim.getCurrentDrawAmps();
    inputs.tempCelsius = 25.0;
  }

  @Override
  public void applyOutputs(IntakeRollersIOOutputs outputs) {

    switch (outputs.mode) {
      case VOLTAGE_CONTROL -> {
        appliedVolts = MathUtil.clamp(outputs.appliedVoltage, -12.0, 12.0);
        sim.setInputVoltage(appliedVolts);
      }

      case BRAKE -> {
        appliedVolts = 0.0;
        sim.setInputVoltage(0.0);
      }

      case COAST -> {
        // For now treat coast same as brake in sim
        appliedVolts = 0.0;
        sim.setInputVoltage(0.0);
      }
    }
  }
}
