package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.RobotState.HoodState;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.subsystems.shooter.hood.HoodIO.HoodIOOutputMode;
import frc.robot.subsystems.shooter.hood.HoodIO.HoodIOOutputs;
import frc.robot.util.FullSubsystem;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Hood extends FullSubsystem {

  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
  private final HoodIOOutputs outputs = new HoodIOOutputs();

  private final Supplier<Pose2d> poseSupplier;
  private final Supplier<ChassisSpeeds> velocitySupplier;

  // make these loggabletunable numbers
  private static final double minAngle = Units.degreesToRadians(10);
  private static final double maxAngle = Units.degreesToRadians(34.5);

  private double goalVoltage = 0.0;
  private double goalAngleRad = 0.0;
  private double goalVelocity = 0.0;

  @AutoLogOutput private HoodState state = HoodState.SEEK_GOAL;

  public Hood(HoodIO io, Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> velocitySupplier) {
    this.io = io;
    this.poseSupplier = poseSupplier;
    this.velocitySupplier = velocitySupplier;
  }

  @Override
  public void periodic() {
    // Update inputs
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);

    // Calculate goal based on state
    switch (state) {
      case SEEK_GOAL -> {
        var shotParams =
            ShotCalculator.calculate(
                poseSupplier.get(),
                velocitySupplier.get(),
                FieldConstants.Hub.topCenterPoint.toTranslation2d());
        goalAngleRad = shotParams.hoodAngle().getRadians();
        goalVelocity = shotParams.hoodVelocity();
      }
      case PASS_BALL -> {
        var shotParams =
            ShotCalculator.calculate(
                poseSupplier.get(),
                velocitySupplier.get(),
                FieldConstants.Depot.depotCenter.toTranslation2d());
        goalAngleRad = shotParams.hoodAngle().getRadians();
        goalVelocity = shotParams.hoodVelocity();
      }
      case FOLD_BACK -> {
        goalAngleRad = 0.0;
        goalVelocity = 0.0;
      }
      case MANUAL -> {}
    }
  }

  @Override
  public void periodicAfterScheduler() {
    // set outputs
    outputs.mode = HoodIOOutputMode.CLOSED_LOOP;

    outputs.positionRad = MathUtil.clamp(goalAngleRad, minAngle, maxAngle);
    outputs.velocityRadsPerSec = goalVelocity;
    outputs.voltage = goalVoltage;

    io.applyOutputs(outputs);

    Logger.recordOutput("Hood/GoalAngleDegrees", Units.radiansToDegrees(goalAngleRad));
    Logger.recordOutput("Hood/GoalVelocity", goalVelocity);
    Logger.recordOutput("Hood/GoalVolts", goalVoltage);
    Logger.recordOutput("Hood/Mode", outputs.mode.toString());
  }

  public void setState(HoodState state) {
    this.state = state;
  }

  public void setGoalParams(double angle, double velocity) {
    goalAngleRad = angle;
    goalVelocity = velocity;
  }

  public void setGoalVoltage(double volts) {
    goalVoltage = volts;
  }

  @AutoLogOutput(key = "Hood/MeasuredAngleDegrees")
  public double getMeasuredAngleDegrees() {
    return Units.radiansToDegrees(inputs.positionRads);
  }

  @AutoLogOutput(key = "Hood/MeasuredAngleRads")
  public double getMeasuredAngleRad() {
    return inputs.positionRads;
  }

  @AutoLogOutput(key = "Hood/MeasuredVelocityRadsPerSec")
  public double getMeasuredVelocityRadsPerSec() {
    return inputs.velocityRadsPerSec;
  }

  @AutoLogOutput(key = "Hood/AtGoal")
  public boolean atGoal() {
    return DriverStation.isEnabled()
        && Math.abs(getMeasuredAngleRad() - goalAngleRad)
            <= Units.degreesToRadians(ShooterConstants.HoodConstants.toleranceDeg.get());
  }

  public Command moveToAngle(double angleRad) {
    return this.runOnce(() -> setGoalParams(angleRad, 0)).andThen(Commands.waitUntil(this::atGoal));
  }

  public Command seekCommand(HoodState state) {
    return this.runOnce(() -> setState(state)).andThen(Commands.waitUntil(this::atGoal));
  }

  public Command seekCommandIndefinite(HoodState state) {
    return this.run(() -> setState(state));
  }
}
