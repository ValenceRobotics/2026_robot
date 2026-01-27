package frc.robot.subsystems.intake.pivot;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class IntakePivot extends SubsystemBase {
  private final IntakePivotIO io;
  private final IntakePivotIOInputsAutoLogged inputs = new IntakePivotIOInputsAutoLogged();

  public IntakePivot(IntakePivotIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("IntakePivot", inputs);
  }

  // **POSITION CONTROL**
  public void stow() {
    io.setPosition(STOWED_POS);
  }

  public void deploy() {
    io.setPosition(DEPLOY_POS);
  }

  public void ground() {
    io.setPosition(GROUND_POS);
  }

  public void setPosition(double positionRot) {
    io.setPosition(positionRot);
  }

  public void stop() {
    io.stop();
  }

  public double getPosition() {
    return inputs.positionRot;
  }

  public double getVelocity() {
    return inputs.velocityRotPerSec;
  }

  public boolean atPosition(double targetRot) {
    return Math.abs(inputs.positionRot - targetRot) < POSITION_TOLERANCE;
  }

  public boolean isStowed() {
    return atPosition(STOWED_POS);
  }

  public boolean isDeployed() {
    return atPosition(DEPLOY_POS);
  }

  public boolean atForwardLimit() {
    return inputs.forwardLimitSwitch;
  }

  public boolean atReverseLimit() {
    return inputs.reverseLimitSwitch;
  }

  public void zeroEncoder() {
    io.zeroToCurrentPos();
  }

  public void setEncoderPosition(double positionRot) {
    io.setCurrentPosition(positionRot);
  }
}
