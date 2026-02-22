package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState.IndexerState;
import frc.robot.subsystems.indexer.IndexerIO.IndexerIOMode;
import frc.robot.subsystems.indexer.IndexerIO.IndexerIOOutputs;
import frc.robot.util.FullSubsystem;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Indexer extends FullSubsystem {
  private final IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();
  private final IndexerIOOutputs outputs = new IndexerIOOutputs();

  private double goalVolts = 0.0;
  @AutoLogOutput private IndexerState state = IndexerState.IDLE;

  public Indexer(IndexerIO io) {
    this.io = io;
    Logger.processInputs("Indexer", inputs);
  }

  @Override
  public void periodic() {

    switch (state) {
      case INDEXING -> goalVolts = IndexerConstants.INDEXER_INDEX_VOLTS;
      case REVERSE -> goalVolts = IndexerConstants.INDEXER_REVERSE_VOLTS;
      case IDLE -> goalVolts = 0.0;
    }

    io.updateInputs(inputs);
    Logger.processInputs("Indexer", inputs);
    Logger.recordOutput("Indexer/GoalVolts", goalVolts);
    Logger.recordOutput("Indexer/velocityRadsPerSec", inputs.velocityRadsPerSec);
    Logger.recordOutput("Indexer/SupplyCurrentAmps", inputs.supplyCurrentAmps);
    Logger.recordOutput("Indexer/StatorCurrentAmps", inputs.statorCurrentAmps);
    Logger.recordOutput("Indexer/Voltage", inputs.appliedVoltage);
  }

  @Override
  public void periodicAfterScheduler() {
    // set outputs
    outputs.mode = IndexerIOMode.VOLTAGE_CONTROL;
    outputs.appliedVoltage = goalVolts;
    io.applyOutputs(outputs);
  }

  public void setState(IndexerState state) {
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

  public Command seekCommand(IndexerState state) {
    return this.runOnce(() -> setState(state));
  }

  public Command seekCommandIndefinite(IndexerState state) {
    return this.run(() -> setState(state));
  }
}
