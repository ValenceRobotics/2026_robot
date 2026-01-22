package frc.robot.subsystems.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class HoodIOSim implements HoodIO {
  private static final DCMotor MOTOR = DCMotor.getNEO(1);

  private final SingleJointedArmSim sim =
      new SingleJointedArmSim(
          MOTOR,
          10.0,
          0.004,
          0.33,
          Units.degreesToRadians(0),
          Units.degreesToRadians(90),
          false,
          Units.degreesToRadians(6.7));

  private double commandedCurrent = 0.0;
  private boolean currentControl = false;

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    double appliedVolts = 0.0;

    if (currentControl) {
      appliedVolts = MOTOR.getVoltage(commandedCurrent, sim.getVelocityRadPerSec());
    }

    // update sim state
    sim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
    sim.update(0.02);

    inputs.motorConnected = true;
    inputs.positionRads = sim.getAngleRads();
    inputs.velocityRadsPerSec = sim.getVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.supplyCurrentAmps = sim.getCurrentDrawAmps();
    inputs.torqueCurrentAmps = commandedCurrent;
  }

  @Override
  public void applyOutputs(HoodIOOutputs outputs) {
    switch (outputs.mode) {
      case BRAKE -> {
        currentControl = false;
        commandedCurrent = 0.0;
      }
      case COAST -> {
        currentControl = true;
        commandedCurrent = 0.0;
      }
      case CLOSED_LOOP -> {
        double posError = outputs.positionRad - sim.getAngleRads();
        double velError = outputs.velocityRadsPerSec - sim.getVelocityRadPerSec();

        commandedCurrent = posError * outputs.kP + velError * outputs.kD;

        currentControl = true;
      }
    }
  }
}
