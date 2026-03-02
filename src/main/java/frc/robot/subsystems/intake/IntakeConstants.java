package frc.robot.subsystems.intake;

import edu.wpi.first.math.util.Units;
import frc.robot.util.LoggedTunableNumber;

public final class IntakeConstants {

  public final class PivotConstants {
    public static final double GEAR_RATIO = 43.2;
    public static final int motorId = 41;

    public static final boolean inverted = false;

    public static final double statorLimitA = 40.0; // safe-ish start
    public static final double peakForwardTorqueA = 60.0; // allow brief bursts
    public static final double peakReverseTorqueA = -60.0;

    public static final double mmCruiseVelocityRotPerSec = 5;
    public static final double mmAccelRotPerSec2 = .5;
    public static final double mmJerkRotPerSec3 = 0.0;

    public static final LoggedTunableNumber kP = new LoggedTunableNumber("IntakePivot/kP", 24);
    public static final LoggedTunableNumber kD = new LoggedTunableNumber("IntakePivot/kD", 0.1);
    public static final LoggedTunableNumber kS = new LoggedTunableNumber("IntakePivot/kS", 0.08);
    public static final LoggedTunableNumber kV = new LoggedTunableNumber("IntakePivot/kV", 0.0);
    public static final LoggedTunableNumber toleranceDeg =
        new LoggedTunableNumber("IntakePivot/ToleranceDeg", 1.0);
  }

  public final class RollersConstants {
    public static final double GEAR_RATIO = 3.0; // Placeholder value, update with actual ratio

    public static final int motorId = 42;
    public static final double INTAKE_VOLTS = 9.6;
    public static final double OUTTAKE_VOLTS = -7.2;
  }

  public static final int MOTOR_ID = 0;

  // Positions
  public static final double STOWED_POS = Units.degreesToRadians(0);
  public static final double GROUND_POS = Units.degreesToRadians(125);

  // Extremes
  public static final double MIN_ANGLE = Units.degreesToRadians(0);
  public static final double MAX_ANGLE = Units.degreesToRadians(134);
}
