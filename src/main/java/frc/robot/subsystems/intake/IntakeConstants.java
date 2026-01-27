package frc.robot.subsystems.intake;

public final class IntakeConstants {
  public static final int MOTOR_ID = 0;

  // Positions
  public static final double STOWED_POS = 0.0;
  public static final double DEPLOY_POS = 0.0;
  public static final double GROUND_POS = 0.0;

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
}
