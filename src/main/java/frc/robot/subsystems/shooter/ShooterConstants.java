package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import frc.robot.util.LoggedTunableNumber;

public class ShooterConstants {

  public class HoodConstants {
    public static final int hoodMotorId = 31;
    public static final int currentLimit = 20;

    public static LoggedTunableNumber kPReal = new LoggedTunableNumber("Hood/kP", 0.065);
    public static LoggedTunableNumber kDReal = new LoggedTunableNumber("Hood/kD", 0.001);
    public static LoggedTunableNumber kGReal = new LoggedTunableNumber("Hood/kG", 0.05);

    public static final double cruiseVelocity = 10; //  m/ sec
    public static final double maxAcceleration = 5; // m/ sec^2
    public static final double allowedError = 0.5;

    public static final double gearRatio = 850.0;
  }

  public class FlywheelConstants {
    public static final int leaderMotorId = 32;
    public static final int followerMotorId = 33;
    public static final int currentLimit = 20;

    public static LoggedTunableNumber kP = new LoggedTunableNumber("Flywheel/kP", 0.065);
    public static LoggedTunableNumber kD = new LoggedTunableNumber("Flywheel/kD", 0.001);
    public static LoggedTunableNumber kG = new LoggedTunableNumber("Flywheel/kG", 0.05);
    public static LoggedTunableNumber kV = new LoggedTunableNumber("Flywheel/kV", 0.0001);
    public static LoggedTunableNumber kS = new LoggedTunableNumber("Flywheel/kS", 0.0);

    public static final double cruiseVelocity = 10; //  m/ sec
    public static final double maxAcceleration = 5; // m/ sec^2
    public static final double allowedError = 0.5;

    public static final double GEAR_RATIO = 1.66;
  }

  public static Transform3d robotToShooter = new Transform3d(0.0, 0.0, 0.44, Rotation3d.kZero);

  public static final InterpolatingTreeMap<Double, Rotation2d> hoodAngleMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);

  public static final InterpolatingDoubleTreeMap flywheelMap = new InterpolatingDoubleTreeMap();

  public static final InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();

  static {

    // pg constants change ltr
    hoodAngleMap.put(1.34, Rotation2d.fromDegrees(19.0));
    hoodAngleMap.put(1.78, Rotation2d.fromDegrees(19.0));
    hoodAngleMap.put(2.17, Rotation2d.fromDegrees(24.0));
    hoodAngleMap.put(2.81, Rotation2d.fromDegrees(27.0));
    hoodAngleMap.put(3.82, Rotation2d.fromDegrees(29.0));
    hoodAngleMap.put(4.09, Rotation2d.fromDegrees(30.0));
    hoodAngleMap.put(4.40, Rotation2d.fromDegrees(31.0));
    hoodAngleMap.put(4.77, Rotation2d.fromDegrees(32.0));
    hoodAngleMap.put(5.60, Rotation2d.fromDegrees(35.0));
    hoodAngleMap.put(6.138, Rotation2d.fromDegrees(43.0));

    flywheelMap.put(1.34, 2100.0);
    flywheelMap.put(2.17, 2200.0);
    flywheelMap.put(3.82, 2500.0);
    flywheelMap.put(5.60, 2900.0);

    timeOfFlightMap.put(1.5, 0.90);
    timeOfFlightMap.put(3.0, 1.05);
    timeOfFlightMap.put(5.5, 1.20);
  }
}
