package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.hood.HoodIO.HoodIOOutputMode;
import frc.robot.subsystems.shooter.hood.HoodIO.HoodIOOutputs;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {

  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
  private final HoodIOOutputs outputs = new HoodIOOutputs();

  // make these loggabletunable numbers
  private static final double minAngle = Units.degreesToRadians(0);
  private static final double maxAngle = Units.degreesToRadians(90);

  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Hood/kP");
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Hood/kD");
  private static final LoggedTunableNumber toleranceDeg =
      new LoggedTunableNumber("Hood/ToleranceDeg");

  private double goalAngleRad = 0.0;
  private double goalVelocity = 0.0;

  public Hood(HoodIO io) {
    this.io = io;

    //  tunable numbers
    kP.initDefault(0.5);
    kD.initDefault(0);
    toleranceDeg.initDefault(10.0); // not implemented in SIM
  }

  @Override
  public void periodic() {
    // Update inputs
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);

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

  public void trackingMode(double distanceMeters) {
    Rotation2d hoodAngle = ShooterConstants.hoodAngleMap.get(distanceMeters);
    setGoalParams(hoodAngle.getRadians(), 0.0);
  }
}
