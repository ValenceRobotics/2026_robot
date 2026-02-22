package frc.robot.subsystems.spindexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState.SpindexerState;
import frc.robot.subsystems.spindexer.SpindexerIO.SpindexerIOMode;
import frc.robot.subsystems.spindexer.SpindexerIO.SpindexerIOOutputs;
import frc.robot.util.FullSubsystem;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Spindexer extends FullSubsystem {
  private final SpindexerIO io;
  private final SpindexerIOInputsAutoLogged inputs = new SpindexerIOInputsAutoLogged();
  private final SpindexerIOOutputs outputs = new SpindexerIOOutputs();

  private double goalVolts = 0.0;
  @AutoLogOutput private SpindexerState state = SpindexerState.IDLE;

  public Spindexer(SpindexerIO io) {
    this.io = io;
    Logger.processInputs("Spindexer", inputs);
  }

  @Override
  public void periodic() {

    switch (state) {
      case INDEXING -> goalVolts = SpindexerConstants.SPINDEXER_INDEX_VOLTS;
      case REVERSE -> goalVolts = SpindexerConstants.SPINDEXER_REVERSE_VOLTS;
      case IDLE -> goalVolts = 0.0;
    }

    io.updateInputs(inputs);
    Logger.processInputs("Spindexer", inputs);
    Logger.recordOutput("Spindexer/GoalVolts", goalVolts);
    Logger.recordOutput("Spindexer/velocityRadsPerSec", inputs.velocityRadsPerSec);
    Logger.recordOutput("Spindexer/SupplyCurrentAmps", inputs.supplyCurrentAmps);
    Logger.recordOutput("Spindexer/StatorCurrentAmps", inputs.statorCurrentAmps);
    Logger.recordOutput("Spindexer/Voltage", inputs.appliedVoltage);
  }

  @Override
  public void periodicAfterScheduler() {
    // set outputs
    outputs.mode = SpindexerIOMode.VOLTAGE_CONTROL;
    outputs.appliedVoltage = goalVolts;
    io.applyOutputs(outputs);
  }

  public void setState(SpindexerState state) {
    this.state = state;
  }

  public void setGoalVolts(double volts) {
    goalVolts = volts;
  }

  @AutoLogOutput(key = "Spindexer/velocityRadsPerSec")
  public double getVelocityRadsPerSec() {
    return inputs.velocityRadsPerSec;
  }

  @AutoLogOutput(key = "Spindexer/MeasuredVoltage")
  public double getAppliedVoltage() {
    return inputs.appliedVoltage;
  }

  public Command seekCommand(SpindexerState state) {
    return this.runOnce(() -> setState(state)); // TODO: ADD UNTIL CONDITION BASED OFF (VOLTAGE?)
  }

  public Command seekCommandIndefinite(SpindexerState state) {
    return this.run(() -> setState(state));
  }
}
