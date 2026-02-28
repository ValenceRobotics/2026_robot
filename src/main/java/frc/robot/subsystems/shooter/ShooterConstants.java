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
    public static LoggedTunableNumber toleranceDeg = new LoggedTunableNumber("Hood/ToleranceDeg", 0.2);
    
    public static LoggedTunableNumber motorStopToleranceDeg = new LoggedTunableNumber("Hood/MotorStopToleranceDeg", 0.1);

    public static final double MIN_ANGLE = Units.degreesToRadians(10);
    public static final double MAX_ANGLE = Units.degreesToRadians(34.5);
    
    public static final double gearRatio = 465.0;
  }

  public class FlywheelConstants {
    public static final int leaderMotorId = 32;
    public static final int followerMotorId = 33;
    public static final int currentLimit = 20;

    public static LoggedTunableNumber kP = new LoggedTunableNumber("Flywheel/kP", 1.0);
    public static LoggedTunableNumber kD = new LoggedTunableNumber("Flywheel/kD", 0.2);
    public static LoggedTunableNumber kG = new LoggedTunableNumber("Flywheel/kG", 0.5);
    public static LoggedTunableNumber kV = new LoggedTunableNumber("Flywheel/kV", 0.1);
    public static LoggedTunableNumber kS = new LoggedTunableNumber("Flywheel/kS", 0.0);

    public static LoggedTunableNumber tolerance = new LoggedTunableNumber("Flywheel/Tolerance", 10.0);

    public static LoggedTunableNumber atGoalDebouncerTime = new LoggedTunableNumber("Flywheel/AtGoalDebounceTime", 0.15);

    public static final double cruiseVelocity = 15; //  m/ sec
    public static final double maxAcceleration = 5; // m/ sec^2
    public static final double allowedError = 0.5;

    public static final double GEAR_RATIO = 1.66;
  }

  public static Transform3d robotToShooter = new Transform3d(0.19, -.16, 0.49, Rotation3d.kZero); // estimated ; might need to change y value to positive 

  public static final InterpolatingTreeMap<Double, Rotation2d> hoodAngleMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);

  public static final InterpolatingDoubleTreeMap flywheelMap = new InterpolatingDoubleTreeMap();

  public static final InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();

  static {

    // pg constants change ltr
    hoodAngleMap.put(1.34, Rotation2d.fromDegrees(12.6331849283));
    hoodAngleMap.put(1.78, Rotation2d.fromDegrees(14.792398641));
    hoodAngleMap.put(2.17, Rotation2d.fromDegrees(16.3097209383));
    hoodAngleMap.put(2.81, Rotation2d.fromDegrees(18.2438637299));
    hoodAngleMap.put(3.82, Rotation2d.fromDegrees(20.3956933844));
    hoodAngleMap.put(4.09, Rotation2d.fromDegrees(20.8448082168));
    hoodAngleMap.put(4.40, Rotation2d.fromDegrees(21.3116778872));
    hoodAngleMap.put(4.77, Rotation2d.fromDegrees(21.8105546923));
    hoodAngleMap.put(5.60, Rotation2d.fromDegrees(22.7460220738));
    hoodAngleMap.put(6.138, Rotation2d.fromDegrees(22.8445734707));

    flywheelMap.put(1.34, 2085.76546142);
    flywheelMap.put(2.17, 2266.99766188);
    flywheelMap.put(3.82, 2615.7114474);
    flywheelMap.put(5.60, 2958.20454555);

    timeOfFlightMap.put(1.5, 0.950496777883);
    timeOfFlightMap.put(3.0, 1.19735698926);
    timeOfFlightMap.put(5.5, 1.52228878711);
  }
}
