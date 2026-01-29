package frc.robot.subsystems.shooter.flywheel;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO.FlywheelIOOutputMode;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO.FlywheelIOOutputs;
import frc.robot.util.FullSubsystem;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends FullSubsystem {
  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();
  private final FlywheelIOOutputs outputs = new FlywheelIOOutputs();

  // tunable stuff
  private static final LoggedTunableNumber tolerance =
      new LoggedTunableNumber("Flywheel/Tolerance", 10.0);
  private static final LoggedTunableNumber atGoalDebounceTime =
      new LoggedTunableNumber("Flywheel/AtGoalDebounceTime", 0.15);

  @AutoLogOutput private boolean running = false;
  @AutoLogOutput private boolean atGoal = false;
  @AutoLogOutput private double goalVelocity = 0.0;

  private Debouncer atGoalDebouncer;

  public Flywheel(FlywheelIO io) {
    this.io = io;
    this.atGoalDebouncer = new Debouncer(atGoalDebounceTime.get(), DebounceType.kRising);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);

    if (atGoalDebounceTime.hasChanged(hashCode())) {
      atGoalDebouncer = new Debouncer(atGoalDebounceTime.get(), DebounceType.kRising);
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

  private void stop() {
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
}
