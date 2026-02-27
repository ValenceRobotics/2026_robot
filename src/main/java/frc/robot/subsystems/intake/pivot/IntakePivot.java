package frc.robot.subsystems.intake.pivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState.IntakePivotState;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.pivot.IntakePivotIO.IntakePivotIOOutputMode;
import frc.robot.subsystems.intake.pivot.IntakePivotIO.IntakePivotIOOutputs;
import frc.robot.util.FullSubsystem;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class IntakePivot extends FullSubsystem {
  private final IntakePivotIO io;
  private final IntakePivotIOInputsAutoLogged inputs = new IntakePivotIOInputsAutoLogged();

  private final IntakePivotIOOutputs outputs = new IntakePivotIOOutputs();
  private static final double minAngle = IntakeConstants.MIN_ANGLE;
  private static final double maxAngle = IntakeConstants.MAX_ANGLE;

  private static final LoggedTunableNumber kP = new LoggedTunableNumber("IntakePivot/kP");
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("IntakePivot/kD");
  private static final LoggedTunableNumber toleranceDeg =
      new LoggedTunableNumber("IntakePivot/ToleranceDeg");

  private double goalPositionRad = IntakeConstants.STOWED_POS;

  @AutoLogOutput private IntakePivotState state = IntakePivotState.DOWN;

  public IntakePivot(IntakePivotIO io) {
    this.io = io;

    toleranceDeg.initDefault(0.5);
    kP.initDefault(0.5);
    kD.initDefault(0);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("IntakePivot", inputs);

    switch (state) {
      case UP -> {
        goalPositionRad = IntakeConstants.STOWED_POS;
      }
      case DOWN -> {
        goalPositionRad = IntakeConstants.GROUND_POS;
      }
    }
  }

  @Override
  public void periodicAfterScheduler() {
    outputs.kP = kP.get();
    outputs.kD = kD.get();

    // set outputs
    outputs.mode = IntakePivotIOOutputMode.CLOSED_LOOP;
    outputs.positionRad = MathUtil.clamp(goalPositionRad, minAngle, maxAngle);

    io.applyOutputs(outputs);

    Logger.recordOutput("IntakePivot/GoalPositionRad", goalPositionRad);
    Logger.recordOutput("IntakePivot/GoalPositionDegrees", Units.radiansToDegrees(goalPositionRad));
    Logger.recordOutput("IntakePivot/Mode", outputs.mode.toString());
  }

  public void setState(IntakePivotState state) {
    this.state = state;
  }

  public void setGoalPositionRad(double positionRad) {
    this.goalPositionRad = MathUtil.clamp(positionRad, minAngle, maxAngle);
  }

  @AutoLogOutput(key = "IntakePivot/MeasuredPositionRad")
  public double getMeasuredPositionRad() {
    return inputs.positionRad;
  }

  @AutoLogOutput(key = "IntakePivot/MeasuredPositionDegrees")
  public double getMeasuredPositionDegrees() {
    return inputs.positionRad;
  }

  @AutoLogOutput(key = "IntakePivot/MeasuredVelocity")
  public double getVelocity() {
    return inputs.velocityRadsPerSec;
  }

  @AutoLogOutput
  public boolean atGoal() {
    return DriverStation.isEnabled()
        && Math.abs(getMeasuredPositionRad() - goalPositionRad)
            <= Math.toRadians(toleranceDeg.get());
  }

  // limit switch implementation; not done yet
  public boolean atForwardLimit() {
    return inputs.forwardLimitSwitch;
  }

  public void zeroEncoder() {
    io.zeroToCurrentPos();
  }

  public void setEncoderPosition(double positionRad) {
    io.setCurrentPosition(positionRad);
  }

  public Command seekCommand(IntakePivotState state) {
    return this.runOnce(() -> setState(state)).andThen(Commands.waitUntil(this::atGoal));
  }

  public Command seekCommandIndefinite(IntakePivotState state) {
    return this.run(() -> setState(state));
  }
}
