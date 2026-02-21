package frc.robot.subsystems.spindexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState.SpindexerState;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Spindexer extends SubsystemBase {
  private final SpindexerIO io;
  private final SpindexerIOInputsAutoLogged inputs = new SpindexerIOInputsAutoLogged();

  @AutoLogOutput private SpindexerState state = SpindexerState.IDLE;

  public Spindexer(SpindexerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Spindexer", inputs);
  }

  public void setState(SpindexerState state) {
    this.state = state;
  }

  public void spin(double speed) {
    io.setVoltage(speed * 12.0);
  }

  public void stop() {
    io.setVoltage(0.0);
  }

  public Command seekCommand(SpindexerState state) {
    return this.runOnce(() -> setState(state)); // TODO: ADD UNTIL CONDITION BASED OFF (VOLTAGE?)
  }

  public Command seekCommandIndefinite(SpindexerState state) {
    return this.run(() -> setState(state)); // TODO: ADD UNTIL CONDITION BASED OFF (VOLTAGE?)
  }
}
