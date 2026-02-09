package frc.robot;

import static frc.robot.subsystems.intake.IntakeConstants.*;
import static frc.robot.subsystems.spindexer.SpindexerConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.pivot.IntakePivot;
import frc.robot.subsystems.intake.rollers.IntakeRollers;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.spindexer.Spindexer;

public class RobotState {
  private final IntakePivot intakePivot;
  private final IntakeRollers intakeRollers;
  private final Flywheel flywheel;
  private final Hood hood;
  private final Spindexer spindexer;

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
}
