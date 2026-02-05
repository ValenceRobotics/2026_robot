package frc.robot.subsystems.intake.rollers;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState.IntakeRollerState;
import frc.robot.subsystems.intake.rollers.IntakeRollersIO.IntakeRollersIOInputs;

public class IntakeRollers extends SubsystemBase {
  private final IntakeRollersIO io;
  private final IntakeRollersIOInputs inputs = new IntakeRollersIO.IntakeRollersIOInputs();

  public IntakeRollers(IntakeRollersIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    // Telemetry
    SmartDashboard.putNumber("IntakeRollers/VelocityRPM", inputs.velocityRPM);
    SmartDashboard.putNumber("IntakeRollers/Current", inputs.currentAmps);
    SmartDashboard.putNumber("IntakeRollers/Voltage", inputs.appliedVolts);
  }

  public void intake() {
    io.setSpeed(INTAKE_SPEED);
  }

  public void outtake() {
    io.setSpeed(OUTTAKE_SPEED);
  }

  public void hold() {
    io.setSpeed(HOLD_SPEED);
  }

  public void feed() {
    io.setSpeed(FEED_SPEED);
  }

  public void stop() {
    io.stop();
  }

  public void setSpeed(double speed) {
    io.setSpeed(speed);
  }

  public void setVoltage(double volts) {
    io.setVoltage(volts);
  }

  public boolean hasGamePieceCurrent() {
    return inputs.currentAmps > HAS_PIECE_CURRENT;
  }

  public boolean isStalled() {
    return inputs.currentAmps > STALL_CURRENT;
  }

  public double getVelocityRPM() {
    return inputs.velocityRPM;
  }

  public double getCurrent() {
    return inputs.currentAmps;
  }

  public boolean isRunning() {
    return Math.abs(inputs.velocityRPM) > 50;
  }

  public boolean isStopped() {
    return Math.abs(inputs.velocityRPM) < 50;
  }

  public Command seekCommand(IntakeRollerState state) {
    return this.runOnce(() -> setSpeed(state.speed))
        .andThen(
            Commands.waitUntil(
                () -> state == IntakeRollerState.STOPPED ? isStopped() : isRunning()));
  }
}
