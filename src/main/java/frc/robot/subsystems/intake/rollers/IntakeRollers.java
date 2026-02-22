package frc.robot.subsystems.intake.rollers;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState.IntakeRollerState;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.rollers.IntakeRollersIO.IntakeRollersIOMode;
import frc.robot.subsystems.intake.rollers.IntakeRollersIO.IntakeRollersIOOutputs;
import frc.robot.util.FullSubsystem;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class IntakeRollers extends FullSubsystem {
  private final IntakeRollersIO io;
  private final IntakeRollersIOInputsAutoLogged inputs = new IntakeRollersIOInputsAutoLogged();
  private final IntakeRollersIOOutputs outputs = new IntakeRollersIOOutputs();

  private double goalVolts = 0.0;
  @AutoLogOutput private IntakeRollerState state = IntakeRollerState.STOPPED;

  public IntakeRollers(IntakeRollersIO io) {
    this.io = io;
    Logger.processInputs("IntakeRollers", inputs);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    switch (state) {
      case INWARD -> goalVolts = IntakeConstants.RollersConstants.INTAKE_VOLTS;
      case OUTWARD -> goalVolts = IntakeConstants.RollersConstants.OUTTAKE_VOLTS;
      case STOPPED -> goalVolts = 0.0;
    }

    io.updateInputs(inputs);
    Logger.processInputs("IntakeRollers", inputs);
    Logger.recordOutput("IntakeRollers/GoalVolts", goalVolts);
    Logger.recordOutput("IntakeRollers/velocityRadsPerSec", inputs.velocityRadsPerSec);
    Logger.recordOutput("IntakeRollers/SupplyCurrentAmps", inputs.supplyCurrentAmps);
    Logger.recordOutput("IntakeRollers/StatorCurrentAmps", inputs.statorCurrentAmps);
    Logger.recordOutput("IntakeRollers/Voltage", inputs.appliedVoltage);
  }

  @Override
  public void periodicAfterScheduler() {
    // set outputs
    outputs.mode = IntakeRollersIOMode.VOLTAGE_CONTROL;
    outputs.appliedVoltage = goalVolts;
    io.applyOutputs(outputs);
  }

  public void setState(IntakeRollerState state) {
    this.state = state;
  }

  public void setGoalVolts(double volts) {
    goalVolts = volts;
  }

  @AutoLogOutput(key = "IntakeRollers/velocityRadsPerSec")
  public double getVelocityRadsPerSec() {
    return inputs.velocityRadsPerSec;
  }

  @AutoLogOutput(key = "IntakeRollers/MeasuredVoltage")
  public double getAppliedVoltage() {
    return inputs.appliedVoltage;
  }

  public Command seekCommand(IntakeRollerState state) {
    return this.runOnce(() -> setState(state)); // TODO: ADD UNTIL CONDITION BASED OFF (VOLTAGE?)
  }

  public Command seekCommandIndefinite(IntakeRollerState state) {
    return this.run(() -> setState(state));
  }
}
