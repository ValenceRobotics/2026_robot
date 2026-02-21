package frc.robot;

import static frc.robot.subsystems.intake.IntakeConstants.GROUND_POS;
import static frc.robot.subsystems.intake.IntakeConstants.INTAKE_SPEED;
import static frc.robot.subsystems.intake.IntakeConstants.OUTTAKE_SPEED;
import static frc.robot.subsystems.intake.IntakeConstants.STOWED_POS;
import static frc.robot.subsystems.spindexer.SpindexerConstants.SPINDEXER_INDEX_VOLTS;
import static frc.robot.subsystems.spindexer.SpindexerConstants.SPINDEXER_REVERSE_VOLTS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.FieldConstants.TrenchSafetyConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.pivot.IntakePivot;
import frc.robot.subsystems.intake.rollers.IntakeRollers;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.spindexer.Spindexer;
import org.littletonrobotics.junction.Logger;

public class RobotState {
  private final IntakePivot intakePivot;
  private final IntakeRollers intakeRollers;
  private final Flywheel flywheel;
  private final Hood hood;
  private final Spindexer spindexer;
  private final Drive drive;

  // Trench safety constants
  private static final double WARNING_ZONE_DISTANCE = TrenchSafetyConstants.WARNING_ZONE_DISTANCE;
  private static final double HARD_ZONE_DISTANCE = TrenchSafetyConstants.HARD_ZONE_DISTANCE;

  public enum IntakePivotState {
    UP(STOWED_POS),
    DOWN(GROUND_POS);

    public final double positionRot;

    IntakePivotState(double positionRot) {
      this.positionRot = positionRot;
    }
  }

  public enum IntakeRollerState {
    INWARD(INTAKE_SPEED),
    OUTWARD(OUTTAKE_SPEED),
    STOPPED(0.0);

    public final double speed;

    IntakeRollerState(double speed) {
      this.speed = speed;
    }
  }

  public enum FlywheelState {
    SEEK_GOAL,
    PASS_BALL,
    STOPPED
  }

  public enum HoodState {
    SEEK_GOAL,
    PASS_BALL,
    FOLD_BACK
  }

  public enum SpindexerState {
    INDEXING(SPINDEXER_INDEX_VOLTS),
    REVERSE(SPINDEXER_REVERSE_VOLTS),
    IDLE(0.0);

    public final double speed;

    SpindexerState(double speed) {
      this.speed = speed;
    }
  }

  public RobotState(RobotContainer container) {
    this.intakePivot = container.intakePivot;
    this.intakeRollers = container.intakeRollers;
    this.flywheel = container.flywheel;
    this.hood = container.hood;
    this.spindexer = container.spindexer;
    this.drive = container.drive;
  }

  private final Trigger trenchWarningTrigger =
      new Trigger(() -> getDistanceToNearestTrench() < WARNING_ZONE_DISTANCE);

  private final Trigger trenchHardTrigger =
      new Trigger(() -> getDistanceToNearestTrench() < HARD_ZONE_DISTANCE);

  public Trigger getTrenchWarningTrigger() {
    return trenchWarningTrigger;
  }

  public Trigger getTrenchHardTrigger() {
    return trenchHardTrigger;
  }

  private double getDistanceToNearestTrench() {
    Pose2d robotPose = drive.getPose();
    double d =
        TrenchSafetyConstants.getClosestPointToNearestTrench(robotPose.getTranslation()).distance();

    Logger.recordOutput("TrenchSafety/DistanceToTrench", d);
    return d;
  }

  /**
   * Seeks one or more mechanism states simultaneously and only terminates when all are reached.
   *
   * @param states (can be any combo of mechanisms)
   * @return command
   */
  public Command seek(Enum<?>... states) {
    Command compound = Commands.none();
    for (Enum<?> state : states) {
      if (state instanceof IntakePivotState s) {
        compound = compound.alongWith(intakePivot.seekCommand(s));
      } else if (state instanceof IntakeRollerState s) {
        compound = compound.alongWith(intakeRollers.seekCommand(s));
      } else if (state instanceof FlywheelState s) {
        compound = compound.alongWith(flywheel.seekCommand(s));
      } else if (state instanceof HoodState s) {
        compound = compound.alongWith(hood.seekCommand(s));
      } else if (state instanceof SpindexerState s) {
        compound = compound.alongWith(spindexer.seekCommand(s));
      }
    }
    return compound;
  }

  public Command seekIndefinite(Enum<?>... states) {
    Command compound = Commands.none();
    for (Enum<?> state : states) {
      if (state instanceof IntakePivotState s) {
        compound = compound.alongWith(intakePivot.seekCommandIndefinite(s));
      } else if (state instanceof IntakeRollerState s) {
        compound = compound.alongWith(intakeRollers.seekCommandIndefinite(s));
      } else if (state instanceof FlywheelState s) {
        compound = compound.alongWith(flywheel.seekCommandIndefinite(s));
      } else if (state instanceof HoodState s) {
        compound = compound.alongWith(hood.seekCommandIndefinite(s));
      } else if (state instanceof SpindexerState s) {
        compound = compound.alongWith(spindexer.seekCommandIndefinite(s));
      }
    }
    return compound.repeatedly();
  }
}
