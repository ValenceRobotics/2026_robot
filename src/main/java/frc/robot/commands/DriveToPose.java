package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import java.util.function.Supplier;

public class DriveToPose extends Command {
  private final Drive drive;
  private final Supplier<Pose2d> targetPose;
  private Supplier<Pose2d> robotPose;

  // pid controllers
  private final ProfiledPIDController xController =
      new ProfiledPIDController(3.0, 0.0, 0.0, new TrapezoidProfile.Constraints(3.0, 3.0));

  private final ProfiledPIDController yController =
      new ProfiledPIDController(3.0, 0.0, 0.0, new TrapezoidProfile.Constraints(3.0, 3.0));

  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          4.0, 0.0, 0.0, new TrapezoidProfile.Constraints(Math.PI * 2.0, Math.PI * 4.0));

  public DriveToPose(Drive drive, Supplier<Pose2d> targetPose) {
    this.drive = drive;
    this.targetPose = targetPose;
    this.robotPose = drive::getPose;

    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(drive);
  }

  public DriveToPose(Drive drive, Supplier<Pose2d> targetPose, Supplier<Pose2d> robotPose) {
    this(drive, targetPose);
    this.robotPose = robotPose;
  }

  @Override
  public void initialize() {
    Pose2d pose = robotPose.get();

    xController.setTolerance(DriveConstants.drivePositionToleranceMeters);
    yController.setTolerance(DriveConstants.drivePositionToleranceMeters);
    thetaController.setTolerance(DriveConstants.driveRotationToleranceRad);

    xController.reset(pose.getX());
    yController.reset(pose.getY());
    thetaController.reset(pose.getRotation().getRadians());
  }

  @Override
  public void execute() {
    Pose2d current = robotPose.get();
    Pose2d target = targetPose.get();

    double vx = xController.atGoal() ? 0.0 : xController.calculate(current.getX(), target.getX());

    double vy = yController.atGoal() ? 0.0 : yController.calculate(current.getY(), target.getY());

    double omega =
        thetaController.atGoal()
            ? 0.0
            : thetaController.calculate(
                current.getRotation().getRadians(), target.getRotation().getRadians());

    drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, current.getRotation()));
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  @Override
  public boolean isFinished() {
    return xController.atGoal() && yController.atGoal() && thetaController.atGoal();
  }
}
