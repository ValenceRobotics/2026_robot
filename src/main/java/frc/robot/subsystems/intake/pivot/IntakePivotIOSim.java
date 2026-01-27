package frc.robot.subsystems.intake.pivot;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class IntakePivotIOSim implements IntakePivotIO {
  private final SingleJointedArmSim pivotSim;
  private double appliedVolts = 0.0;
  private double targetPositionRot = 0.0;

  public IntakePivotIOSim() {
    pivotSim =
        new SingleJointedArmSim(
            DCMotor.getFalcon500(1),
            GEAR_RATIO,
            0.1,
            0.5,
            Units.degreesToRadians(-5),
            Units.degreesToRadians(100),
            true,
            Units.degreesToRadians(STOWED_POS));
  }

  @Override
  public void updateInputs(IntakePivotIOInputs inputs) {
    pivotSim.update(0.02); // 20ms loop time

    // Motor values (before gearing)
    inputs.positionRot = Units.radiansToRotations(pivotSim.getAngleRads()) * GEAR_RATIO;
    inputs.velocityRotPerSec =
        Units.radiansToRotations(pivotSim.getVelocityRadPerSec()) * GEAR_RATIO;

    // Mechanism values (after gearing)
    inputs.positionRot = Units.radiansToRotations(pivotSim.getAngleRads());
    inputs.velocityRotPerSec = Units.radiansToRotations(pivotSim.getVelocityRadPerSec());

    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = pivotSim.getCurrentDrawAmps();
    inputs.tempCelsius = 25.0;

    inputs.forwardLimitSwitch = pivotSim.getAngleRads() > Units.degreesToRadians(95);
    inputs.reverseLimitSwitch = pivotSim.getAngleRads() < Units.degreesToRadians(2);
  }

  @Override
  public void setPosition(double positionRot) {
    targetPositionRot = positionRot;
    // Only kP for sim
    double currentRot = Units.radiansToRotations(pivotSim.getAngleRads());
    double error = targetPositionRot - currentRot;
    setVoltage(error * kP * 12.0);
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = volts;
    pivotSim.setInputVoltage(volts);
  }

  @Override
  public void stop() {
    setVoltage(0);
  }

  @Override
  public void zeroToCurrentPos() {
    pivotSim.setState(0, pivotSim.getVelocityRadPerSec());
  }

  @Override
  public void setCurrentPosition(double positionRot) {
    pivotSim.setState(Units.rotationsToRadians(positionRot), pivotSim.getVelocityRadPerSec());
  }
}
