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
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.subsystems.shooter.hood.HoodIO.HoodIOOutputMode;
import frc.robot.subsystems.shooter.hood.HoodIO.HoodIOOutputs;
import frc.robot.util.FullSubsystem;
import frc.robot.util.LoggedTunableNumber;
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
  private static final double minAngle = Units.degreesToRadians(0);
  private static final double maxAngle = Units.degreesToRadians(90);

  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Hood/kP");
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Hood/kD");
  private static final LoggedTunableNumber toleranceDeg =
      new LoggedTunableNumber("Hood/ToleranceDeg");

  private double goalAngleRad = 0.0;
  private double goalVelocity = 0.0;

  @AutoLogOutput private HoodState state = HoodState.FOLD_BACK;

  public Hood(HoodIO io, Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> velocitySupplier) {
    this.io = io;
    this.poseSupplier = poseSupplier;
    this.velocitySupplier = velocitySupplier;

    //  tunable numbers
    kP.initDefault(0.5);
    kD.initDefault(0);
    toleranceDeg.initDefault(10.0);
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
    }
  }

  @Override
  public void periodicAfterScheduler() {
    // set outputs
    outputs.mode = HoodIOOutputMode.CLOSED_LOOP;
    outputs.positionRad = MathUtil.clamp(goalAngleRad, minAngle, maxAngle);
    outputs.velocityRadsPerSec = goalVelocity;
    outputs.kP = kP.get();
    outputs.kD = kD.get();

    io.applyOutputs(outputs);

    Logger.recordOutput("Hood/GoalAngleRad", goalAngleRad);
    Logger.recordOutput("Hood/GoalVelocity", goalVelocity);
    Logger.recordOutput("Hood/Mode", outputs.mode.toString());
    Logger.recordOutput("Hood/Kp", kP.get());
  }

  public void setState(HoodState state) {
    this.state = state;
  }

  public void setGoalParams(double angle, double velocity) {
    goalAngleRad = angle;
    goalVelocity = velocity;
  }

  @AutoLogOutput(key = "Hood/MeasuredAngleRads")
  public double getMeasuredAngleRad() {
    return inputs.positionRads;
  }

  @AutoLogOutput(key = "Hood/MeasuredVelocityRadsPerSec")
  public double getMeasuredVelocityRadsPerSec() {
    return inputs.velocityRadsPerSec;
  }

  @AutoLogOutput
  public boolean atGoal() {
    return DriverStation.isEnabled()
        && Math.abs(getMeasuredAngleRad() - goalAngleRad)
            <= Units.degreesToRadians(toleranceDeg.get());
  }

  public Command seekCommand(HoodState state) {
    return this.runOnce(() -> setState(state)).andThen(Commands.waitUntil(this::atGoal));
  }
}
