package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.FieldConstants;
import frc.robot.util.geometry.AllianceFlipUtil;

// to do: add hub offset zone for shots to aim at to make our heading pose less sensitive.
public class ShotCalculator {

  public record ShotParams(
      Rotation2d hoodAngle,
      double hoodVelocity,
      double flywheelRPM,
      Rotation2d robotHeading,
      double distanceMeters) {}

  private static double lastHood = 0.0;
  private static double lastDistance = Double.NaN;
  private static final double LOOP_PERIOD = 0.02;

  public static ShotParams calculate(Pose2d pose, ChassisSpeeds fieldVelocity) {
    return calculate(pose, fieldVelocity, FieldConstants.Hub.topCenterPoint.toTranslation2d());
  }

  public static ShotParams calculate(
      Pose2d pose, ChassisSpeeds fieldVelocity, Translation2d target) {

    target = AllianceFlipUtil.apply(target);

    Transform2d robotToShooter2d =
        new Transform2d(
            ShooterConstants.robotToShooter.getTranslation().toTranslation2d(),
            ShooterConstants.robotToShooter.getRotation().toRotation2d());

    Pose2d shooterPose = pose.transformBy(robotToShooter2d);
    Translation2d shooterPos = shooterPose.getTranslation();

    double distance = shooterPos.getDistance(target);
    double tof = ShooterConstants.timeOfFlightMap.get(distance);

    Translation2d futureShooterPos =
        shooterPos.plus(
            new Translation2d(
                fieldVelocity.vxMetersPerSecond * tof, fieldVelocity.vyMetersPerSecond * tof));

    double futureDistance = futureShooterPos.getDistance(target);

    Rotation2d hood = ShooterConstants.hoodAngleMap.get(futureDistance);
    double rpm = ShooterConstants.flywheelMap.get(futureDistance);

    double hoodVel = 0.0;
    if (!Double.isNaN(lastDistance)) {
      double nextHood = ShooterConstants.hoodAngleMap.get(futureDistance).getRadians();
      hoodVel = (nextHood - lastHood) / LOOP_PERIOD;
    }

    lastHood = hood.getRadians();
    lastDistance = futureDistance;

    Rotation2d robotHeading = target.minus(futureShooterPos).getAngle();

    return new ShotParams(hood, hoodVel, rpm, robotHeading, futureDistance);
  }
}
