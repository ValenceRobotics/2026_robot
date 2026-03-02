package frc.robot.subsystems.intake.pivot;

import static frc.robot.subsystems.intake.IntakeConstants.STOWED_POS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.shooter.ShooterConstants.HoodConstants;

public class IntakePivotIOSim implements IntakePivotIO {
  private final SingleJointedArmSim pivotSim;
  private double appliedVolts = 0.0;

  public IntakePivotIOSim() {

    pivotSim =
        new SingleJointedArmSim(
            DCMotor.getFalcon500(1),
            HoodConstants.gearRatio,
            0.1,
            0.5,
            Units.degreesToRadians(-5),
            Units.degreesToRadians(100),
            false,
            Units.degreesToRadians(STOWED_POS));
  }

  @Override
  public void updateInputs(IntakePivotIOInputs inputs) {
    // Motor values (before gearing)
    inputs.positionRad = pivotSim.getAngleRads();
    inputs.velocityRadsPerSec = pivotSim.getVelocityRadPerSec();

    inputs.appliedVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);
    pivotSim.setInputVoltage(inputs.appliedVolts);
    pivotSim.update(0.02);

    inputs.currentAmps = pivotSim.getCurrentDrawAmps();
    inputs.tempCelsius = 25.0;

    inputs.forwardLimitSwitch = pivotSim.getAngleRads() > Units.degreesToRadians(95);
    inputs.reverseLimitSwitch = pivotSim.getAngleRads() < Units.degreesToRadians(2);
  }

  @Override
  public void applyOutputs(IntakePivotIOOutputs outputs) {
    switch (outputs.mode) {
      case BRAKE -> {
        appliedVolts = 0.0;
      }
      case COAST -> {
        appliedVolts = 0.0;
      }
      case CLOSED_LOOP -> {
        double posError = outputs.positionRad - pivotSim.getAngleRads();
        double velError = outputs.velocityRadsPerSec - pivotSim.getVelocityRadPerSec();

        double volts =
            (posError * IntakeConstants.PivotConstants.kP.get())
                + (velError * IntakeConstants.PivotConstants.kD.get());

        appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
      }
    }
  }
}
