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

    public static final double mmCruiseVelocityRotPerSec = 0.2;
    public static final double mmAccelRotPerSec2 = .1;
    public static final double mmJerkRotPerSec3 = 0.0;

    public static final LoggedTunableNumber kP = new LoggedTunableNumber("IntakePivot/kP", 40.0);
    public static final LoggedTunableNumber kD = new LoggedTunableNumber("IntakePivot/kD", 2.0);
    public static final LoggedTunableNumber kS = new LoggedTunableNumber("IntakePivot/kS", 0.0);
    public static final LoggedTunableNumber kV = new LoggedTunableNumber("IntakePivot/kV", 0.0);
  }

  public final class RollersConstants {
    public static final double GEAR_RATIO = 3.0; // Placeholder value, update with actual ratio

    public static final int motorId = 42;
    public static final double INTAKE_VOLTS = 9.6;
    public static final double OUTTAKE_VOLTS = -7.2;
  }

  public static final int MOTOR_ID = 0;

  // Positions
  public static final double STOWED_POS = Units.degreesToRadians(85);
  public static final double GROUND_POS = Units.degreesToRadians(0);

  // Extremes
  public static final double MIN_ANGLE = Units.degreesToRadians(0);
  public static final double MAX_ANGLE = Units.degreesToRadians(90);

  // Motion Magic
  public static final double kP = 0.1;
  public static final double kD = 0.0;
  public static final double kF = 0.0; // Feedforward
  public static final double kG = 0.05; // Gravity Compensation (Unsure if nescessary)

  // Motic Magic Profile
  public static final double MAX_VELOCITY = Double.POSITIVE_INFINITY;
  public static final double MAX_ACCEL = 10.0;
  public static final int CURVE_STRENGTH = 0; // Curve (if nescessary)

  // Tolerance
  public static final double POSITION_TOLERANCE = 0.05;

  // Ratio
  public static final double GEAR_RATIO = 100.0; // Motor rotations per pivot rotation

  // Limits
  public static final double CURRENT_LIMIT = 40;
  public static final double LIMIT_FWD = 3.0;
  public static final double LIMIT_REV = -0.1;

  public static final double HOLD_SPEED = 0.1; // Keep piece secured
  public static final double FEED_SPEED = 0.5; // Feed to next mechanism

  // Current monitoring
  public static final double STALL_CURRENT = 30.0; // Current spike indicates piece acquired
  public static final double HAS_PIECE_CURRENT = 15.0; // Sustained current with piece
}
