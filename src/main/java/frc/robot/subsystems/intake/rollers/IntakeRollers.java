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

  private double goalSpeed = 0.0;
  @AutoLogOutput private IntakeRollerState state = IntakeRollerState.STOPPED;

  public IntakeRollers(IntakeRollersIO io) {
    this.io = io;
    Logger.processInputs("IntakeRollers", inputs);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    switch (state) {
      case INWARD -> {
        goalSpeed = IntakeConstants.INTAKE_SPEED;
      }
      case OUTWARD -> {
        goalSpeed = IntakeConstants.OUTTAKE_SPEED;
      }
      case STOPPED -> {
        goalSpeed = 0.0;
      }
    }

    Logger.recordOutput("IntakeRollers/GoalSpeed", goalSpeed);
    Logger.recordOutput("IntakeRollers/VelocityRPM", inputs.velocityRPM);
    Logger.recordOutput("IntakeRollers/Current", inputs.currentAmps);
    Logger.recordOutput("IntakeRollers/Voltage", inputs.appliedVolts);
  }

  @Override
  public void periodicAfterScheduler() {
    // set outputs
    outputs.mode = IntakeRollersIOMode.VOLTAGE_CONTROL;
    outputs.appliedVoltage = goalSpeed * 12.0;
    io.applyOutputs(outputs);
  }

  public void setState(IntakeRollerState state) {
    this.state = state;
  }

  public void setGoalSpeed(double speed) {
    goalSpeed = speed;
  }

  @AutoLogOutput(key = "IntakeRollers/MeasuredVelocityRPM")
  public double getVelocityRPM() {
    return inputs.velocityRPM;
  }

  @AutoLogOutput(key = "IntakeRollers/MeasuredVoltage")
  public double getAppliedVoltage() {
    return inputs.appliedVolts;
  }

  public Command seekCommand(IntakeRollerState state) {
    return this.runOnce(() -> setState(state)); // TODO: ADD UNTIL CONDITION BASED OFF (VOLTAGE?)
  }

  public Command seekCommandIndefinite(IntakeRollerState state) {
    return this.run(() -> setState(state));
  }
}
