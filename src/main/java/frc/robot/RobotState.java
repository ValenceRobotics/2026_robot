// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage

package frc.robot;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import java.util.NoSuchElementException;
import java.util.Optional;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.intake.pivot.IntakePivot;
import frc.robot.subsystems.intake.rollers.IntakeRollers;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.hood.Hood;
import lombok.Getter;
import lombok.Setter;

public class RobotState {
  private final IntakePivot intakePivot;
  private final IntakeRollers intakeRollers;
  private final Flywheel flywheel;
  private final Hood hood;

  // Pose Constants
  private static final double poseBufferSec = 2.0;
  private static final Matrix<N3, N1> odometryStateStdevs =
    new Matrix<>(VecBuilder.fill(.003, .003, .002));

  @Getter @AutoLogOutput private Pose2d odomPose = Pose2d.kZero;
  @Getter @AutoLogOutput private Pose2d estimatedPose = Pose2d.kZero;
  private final TimeInterpolatableBuffer<Pose2d> poseBuffer =
    TimeInterpolatableBuffer.createBuffer(poseBufferSec);
  private final Matrix<N3, N1> qStDevs = new Matrix<>(Nat.N3(), Nat.N1());

  // Odom Fields
  private final SwerveDriveKinematics kinematics;
  private SwerveModulePosition[] lastWheelPositions =
    new SwerveModulePosition[] {
      new SwerveModulePosition(),
      new SwerveModulePosition(),
      new SwerveModulePosition(),
      new SwerveModulePosition(),
    };

  private Rotation2d gyroOffset = Rotation2d.kZero;

  @Getter @Setter private ChassisSpeeds robotVelocity = new ChassisSpeeds();

  private static RobotState instance;

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

  public RobotState(RobotContainer container) {
    this.intakePivot = container.intakePivot;
    this.intakeRollers = container.intakeRollers;
    this.flywheel = container.flywheel;
    this.hood = container.hood;

    for (int i=0; i < 3; i++) {
      qStDevs.set(i, 0, Math.pow(qStDevs.get(i,0), 2));
    };
    kinematics = new SwerveDriveKinematics(DriveConstants.moduleTranslations);
  }

  public static RobotState getInstance(RobotContainer container) {
    if (instance == null) instance = new RobotState(container);
    return instance;
  }
  
  public void resetPose(Pose2d pose) {
    gyroOffset = pose.getRotation().minus(odomPose.getRotation().minus(gyroOffset));
    estimatedPose = pose;
    odomPose = pose;
    poseBuffer.clear();
  }
  
  public Rotation2d getRotation() {
    return estimatedPose.getRotation();
  }

  public ChassisSpeeds getFieldVelocity() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(robotVelocity, getRotation());
  }

  /** Adds a new odometry sample from the drive subsystem. */
  public void addOdometryObservation(OdometryObservation observation) {
    // Update odometry pose
    Twist2d twist = kinematics.toTwist2d(lastWheelPositions, observation.wheelPositions());
    lastWheelPositions = observation.wheelPositions();
    Pose2d lastOdometryPose = odomPose;
    odomPose = odomPose.exp(twist);

    // Replace odometry pose with gyro if present
    observation.gyroAngle.ifPresent(
        gyroAngle -> {
          // Add offset to measured angle
          Rotation2d angle = gyroAngle.plus(gyroOffset);
          odomPose = new Pose2d(odomPose.getTranslation(), angle);
        });

    // Add pose to buffer at timestamp
    poseBuffer.addSample(observation.timestamp(), odomPose);

    // Apply odometry delta to vision pose estimate
    Twist2d finalTwist = lastOdometryPose.log(odomPose);
    estimatedPose = estimatedPose.exp(finalTwist);
  }

  public void addVisionObservation(VisionObservation observation) {
    // If measurement is old enough to be outside the pose buffer's timespan, skip.
    try {
      if (poseBuffer.getInternalBuffer().lastKey() - poseBufferSec > observation.timestamp()) {
        return;
      }
    } catch (NoSuchElementException ex) {
      return;
    }

    // Get odometry based pose at timestamp
    var sample = poseBuffer.getSample(observation.timestamp());
    if (sample.isEmpty()) {
      // exit if not there
      return;
    }

    // Calculate transforms between odometry pose and vision sample pose
    var sampleToOdometryTransform = new Transform2d(sample.get(), odomPose);
    var odometryToSampleTransform = new Transform2d(odomPose, sample.get());

    // Shift estimated pose backwards to sample time
    Pose2d estimateAtTime = estimatedPose.plus(odometryToSampleTransform);

    // Calculate 3 x 3 vision matrix
    var r = new double[3];
    for (int i = 0; i < 3; ++i) {
      r[i] = observation.stdDevs().get(i, 0) * observation.stdDevs().get(i, 0);
    }

    // Solve for closed form Kalman gain for continuous Kalman filter with A = 0
    // and C = I. See wpimath/algorithms.md.
    Matrix<N3, N3> visionK = new Matrix<>(Nat.N3(), Nat.N3());
    for (int row = 0; row < 3; ++row) {
      double stdDev = qStDevs.get(row, 0);
      if (stdDev == 0.0) {
        visionK.set(row, row, 0.0);
      } else {
        visionK.set(row, row, stdDev / (stdDev + Math.sqrt(stdDev * r[row])));
      }
    }

    // Calculate the transform from the shifted estimate to the observation pose
    Transform2d transform = new Transform2d(estimateAtTime, observation.visionPose().toPose2d());

    // Scale the transform by the Kalman gain
    var kTimesTransform =
        visionK.times(
            VecBuilder.fill(
                transform.getX(), transform.getY(), transform.getRotation().getRadians()));
    Transform2d scaledTransform =
        new Transform2d(
            kTimesTransform.get(0, 0),
            kTimesTransform.get(1, 0),
            Rotation2d.fromRadians(kTimesTransform.get(2, 0)));

    // Recalculate the current estimate by applying the scaled transform to the old estimate
    // then shifting forwards using odometry data
    estimatedPose = estimateAtTime.plus(scaledTransform).plus(sampleToOdometryTransform);
  }

  public record OdometryObservation(
    double timestamp, SwerveModulePosition[] wheelPositions, Optional<Rotation2d> gyroAngle) {};
  
  public record VisionObservation(
    double timestamp, Pose3d visionPose, Matrix<N3, N1> stdDevs) {};

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
      }
    }
    return compound;
  }
}
