package frc.robot.subsystems.shooter.flywheel;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.RobotState.FlywheelState;
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO.FlywheelIOOutputMode;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO.FlywheelIOOutputs;
import frc.robot.util.FullSubsystem;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends FullSubsystem {
  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();
  private final FlywheelIOOutputs outputs = new FlywheelIOOutputs();

  private final Supplier<Pose2d> poseSupplier;
  private final Supplier<ChassisSpeeds> velocitySupplier;

  // tunable stuff
  private static final LoggedTunableNumber tolerance =
      new LoggedTunableNumber("Flywheel/Tolerance", 10.0);
  private static final LoggedTunableNumber atGoalDebounceTime =
      new LoggedTunableNumber("Flywheel/AtGoalDebounceTime", 0.15);

  @AutoLogOutput private boolean running = false;
  @AutoLogOutput private boolean atGoal = false;
  @AutoLogOutput private double goalVelocity = 0.0;

  @AutoLogOutput private FlywheelState state = FlywheelState.STOPPED;

  private Debouncer atGoalDebouncer;

  public Flywheel(
      FlywheelIO io, Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> velocitySupplier) {
    this.io = io;
    this.poseSupplier = poseSupplier;
    this.velocitySupplier = velocitySupplier;
    this.atGoalDebouncer = new Debouncer(atGoalDebounceTime.get(), DebounceType.kRising);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);

    if (atGoalDebounceTime.hasChanged(hashCode())) {
      atGoalDebouncer = new Debouncer(atGoalDebounceTime.get(), DebounceType.kRising);
    }

    // Calculate goal based on state
    switch (state) {
      case SEEK_GOAL -> {
        var shotParams =
            ShotCalculator.calculate(
                poseSupplier.get(),
                velocitySupplier.get(),
                FieldConstants.Hub.topCenterPoint.toTranslation2d());
        goalVelocity = shotParams.flywheelRPM() * 2.0 * Math.PI / 60.0; // RPM to rad/s
        running = true;
      }
      case PASS_BALL -> {
        var shotParams =
            ShotCalculator.calculate(
                poseSupplier.get(),
                velocitySupplier.get(),
                FieldConstants.Depot.depotCenter.toTranslation2d());
        goalVelocity = shotParams.flywheelRPM() * 2.0 * Math.PI / 60.0; // RPM to rad/s
        running = true;
      }
      case STOPPED -> {
        goalVelocity = 0.0;
        running = false;
      }
    }

    boolean inTolerance =
        running && Math.abs(inputs.velocityRadsPerSec - goalVelocity) < tolerance.get();
    atGoal = atGoalDebouncer.calculate(inTolerance);
  }

  @Override
  public void periodicAfterScheduler() {
    if (running) {
      outputs.mode = FlywheelIOOutputMode.VELOCITY;
      outputs.velocityRadsPerSec = goalVelocity;
    } else {
      outputs.mode = FlywheelIOOutputMode.COAST;
      outputs.velocityRadsPerSec = 0.0;
    }

    Logger.recordOutput("Flywheel/Mode", outputs.mode);
    Logger.recordOutput("Flywheel/GoalVelocity", goalVelocity);
    Logger.recordOutput("Flywheel/GoalRPM", goalVelocity * 60.0 / (2.0 * Math.PI));

    io.applyOutputs(outputs);
  }

  @AutoLogOutput(key = "Flywheel/MeasuredVelocity")
  public double getMeasuredAngleRad() {
    return inputs.velocityRadsPerSec;
  }

  @AutoLogOutput(key = "Flywheel/MeasuredVelocityRPM")
  public double getMeasuredVelocityRPM() {
    return inputs.velocityRadsPerSec * 60.0 / (2.0 * Math.PI);
  }

  public void setState(FlywheelState state) {
    this.state = state;
  }

  private void stop() {
    state = FlywheelState.STOPPED;
    running = false;
    goalVelocity = 0.0;
    atGoal = false;
  }

  public double getVelocity() {
    return inputs.velocityRadsPerSec;
  }

  public void setGoalVelocity(double velocityRadsPerSec) {
    this.goalVelocity = velocityRadsPerSec;
    this.running = true;
  }

  public Command runVelocityCommand(DoubleSupplier velocity) {
    return this.runEnd(() -> setGoalVelocity(velocity.getAsDouble()), this::stop);
  }

  public Command stopCommand() {
    return this.runOnce(this::stop);
  }

  public boolean atGoal() {
    return atGoal;
  }

  public Command seekCommand(FlywheelState state) {
    if (state == FlywheelState.STOPPED) {
      return this.runOnce(this::stop).andThen(Commands.waitUntil(() -> !running));
    }
    return this.runOnce(() -> setState(state)).andThen(Commands.waitUntil(this::atGoal));
  }
}
