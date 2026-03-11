package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.util.Units;
import frc.robot.util.LoggedTunableNumber;

public class ShooterConstants {

  public class HoodConstants {
    public static final int hoodMotorId = 31;
    public static final int currentLimit = 20;

    public static LoggedTunableNumber kPReal = new LoggedTunableNumber("Hood/kP", 8.0);
    public static LoggedTunableNumber kDReal = new LoggedTunableNumber("Hood/kD", 0);
    public static LoggedTunableNumber kGReal = new LoggedTunableNumber("Hood/kG", 0);
    public static LoggedTunableNumber toleranceDeg =
        new LoggedTunableNumber("Hood/ToleranceDeg", 0.6);

    public static LoggedTunableNumber motorStopToleranceDeg =
        new LoggedTunableNumber("Hood/MotorStopToleranceDeg", 0.1);

    public static final double MIN_ANGLE = Units.degreesToRadians(10);
    public static final double MAX_ANGLE = Units.degreesToRadians(33);

    public static final double gearRatio = 465.0;
  }

  public class FlywheelConstants {
    public static final int leaderMotorId = 32;
    public static final int followerMotorId = 33;
    public static final int currentLimit = 20;

    public static LoggedTunableNumber kP = new LoggedTunableNumber("Flywheel/kP", 0.0);
    public static LoggedTunableNumber kD = new LoggedTunableNumber("Flywheel/kD", 0.0);
    public static LoggedTunableNumber kG = new LoggedTunableNumber("Flywheel/kG", 0.0);
    public static LoggedTunableNumber kV = new LoggedTunableNumber("Flywheel/kV", 0.19424);
    public static LoggedTunableNumber kS = new LoggedTunableNumber("Flywheel/kS", 0.17831);

    public static LoggedTunableNumber tolerance =
        new LoggedTunableNumber("Flywheel/Tolerance", 500.0);

    public static LoggedTunableNumber atGoalDebouncerTime =
        new LoggedTunableNumber("Flywheel/AtGoalDebounceTime", 0.15);

    public static final double GEAR_RATIO = 1.66;
  }

  public static Transform3d robotToShooter =
      new Transform3d(
          0.19,
          -.16,
          0.49,
          Rotation3d.kZero); // estimated ; might need to change y value to positive

  public static final InterpolatingTreeMap<Double, Rotation2d> hoodAngleMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);

  public static final InterpolatingDoubleTreeMap flywheelMap = new InterpolatingDoubleTreeMap();

  public static final InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();

  static {

    // pg constants change ltr
    hoodAngleMap.put(1.2192, Rotation2d.fromDegrees(12));
    hoodAngleMap.put(1.524, Rotation2d.fromDegrees(17.0));
    hoodAngleMap.put(1.98, Rotation2d.fromDegrees(19));
    hoodAngleMap.put(2.4384, Rotation2d.fromDegrees(21));
    hoodAngleMap.put(2.642, Rotation2d.fromDegrees(23));
    hoodAngleMap.put(5.0, Rotation2d.fromDegrees(32));

    flywheelMap.put(1.2192, 2000.0);
    flywheelMap.put(1.524, 2100.0);
    flywheelMap.put(1.9812, 2300.0);
    flywheelMap.put(2.4384, 2300.0);
    flywheelMap.put(2.642, 2400.0);
    flywheelMap.put(5.0, 2700.0);

    timeOfFlightMap.put(1.5, 0.950496777883);
    timeOfFlightMap.put(3.0, 1.19735698926);
    timeOfFlightMap.put(5.5, 1.52228878711);
  }
}
