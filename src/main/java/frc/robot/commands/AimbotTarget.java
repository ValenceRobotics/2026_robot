package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.subsystems.shooter.hood.Hood;

/* todo: flywheels tuff */
public class AimbotTarget extends Command {
  private final Hood hood;
  private final Drive drive;

  public AimbotTarget(Hood hood, Drive drive) {
    this.hood = hood;
    this.drive = drive;
    addRequirements(hood);
  }

  @Override
  public void execute() {
    Pose2d pose = drive.getPose();
    ChassisSpeeds fieldVel = drive.getFieldVelocity();

    var params = ShotCalculator.calculate(pose, fieldVel);

    hood.setGoalParams(params.hoodAngle().getRadians(), params.hoodVelocity());
    drive.setAimbotHeading(params.robotHeading());
    // flywheel.setGoalRPM(params.flywheelRPM());
  }
}
