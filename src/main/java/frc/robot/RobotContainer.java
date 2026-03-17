// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.cameraFName;
import static frc.robot.subsystems.vision.VisionConstants.cameraLName;
import static frc.robot.subsystems.vision.VisionConstants.cameraRName;
import static frc.robot.subsystems.vision.VisionConstants.robotToCameraF;
import static frc.robot.subsystems.vision.VisionConstants.robotToCameraL;
import static frc.robot.subsystems.vision.VisionConstants.robotToCameraR;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotState.FlywheelState;
import frc.robot.RobotState.HoodState;
import frc.robot.RobotState.IndexerState;
import frc.robot.RobotState.IntakePivotState;
import frc.robot.RobotState.IntakeRollerState;
import frc.robot.RobotState.SpindexerState;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOReal;
import frc.robot.subsystems.intake.pivot.IntakePivot;
import frc.robot.subsystems.intake.pivot.IntakePivotIO;
import frc.robot.subsystems.intake.pivot.IntakePivotIOSim;
import frc.robot.subsystems.intake.pivot.IntakePivotIOTalonFX;
import frc.robot.subsystems.intake.rollers.IntakeRollers;
import frc.robot.subsystems.intake.rollers.IntakeRollersIO;
import frc.robot.subsystems.intake.rollers.IntakeRollersIOSim;
import frc.robot.subsystems.intake.rollers.IntakeRollersIOTalonFX;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOTalonFX;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.hood.HoodIO;
import frc.robot.subsystems.shooter.hood.HoodIOReal;
import frc.robot.subsystems.shooter.hood.HoodIOSim;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.spindexer.SpindexerIOReal;
import frc.robot.subsystems.spindexer.SpindexerIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  final Drive drive;
  final Vision vision;
  final Hood hood;
  final IntakePivot intakePivot;
  final IntakeRollers intakeRollers;
  final Flywheel flywheel;
  final Spindexer spindexer;
  final Indexer indexer;
  // final LED led;

  // Robot state
  final RobotState robotState;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(1);
  private final CommandXboxController manualController = new CommandXboxController(2);
  private final CommandGenericHID keyboard = new CommandGenericHID(0); // Keyboard 0 on port 0

  private static LoggedTunableNumber tuneHoodDeg =
      new LoggedTunableNumber("tuning/hood angle", 10.0);
  private static LoggedTunableNumber tuneFlywheelRPM =
      new LoggedTunableNumber("tuning/flywheel RPM", 1000.0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.getMode()) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3));

        // vision =
        // new Vision(
        // drive::addVisionMeasurement,
        // new VisionIOLimelight(camera0Name, drive::getRotation),
        // new VisionIOLimelight(camera1Name, drive::getRotation));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(cameraFName, robotToCameraF),
                new VisionIOPhotonVision(cameraLName, robotToCameraL),
                new VisionIOPhotonVision(cameraRName, robotToCameraR));

        this.hood = new Hood(new HoodIOReal(), drive::getPose, drive::getFieldVelocity);
        this.indexer = new Indexer(new IndexerIOReal());
        this.intakePivot =
            new IntakePivot(
                new IntakePivotIOTalonFX()); // if this breaks change it back to iosim here for now
        this.intakeRollers = new IntakeRollers(new IntakeRollersIOTalonFX());
        this.flywheel =
            new Flywheel(new FlywheelIOTalonFX(), drive::getPose, drive::getFieldVelocity);
        this.spindexer = new Spindexer(new SpindexerIOReal());
        // this.led = new LED();
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(cameraFName, robotToCameraF, drive::getPose),
                new VisionIOPhotonVisionSim(cameraLName, robotToCameraL, drive::getPose),
                new VisionIOPhotonVisionSim(cameraRName, robotToCameraR, drive::getPose));

        hood = new Hood(new HoodIOSim(), drive::getPose, drive::getFieldVelocity);
        intakePivot = new IntakePivot(new IntakePivotIOSim());
        intakeRollers = new IntakeRollers(new IntakeRollersIOSim());
        flywheel = new Flywheel(new FlywheelIO() {}, drive::getPose, drive::getFieldVelocity);
        spindexer = new Spindexer(new SpindexerIOSim() {});
        this.indexer = new Indexer(new IndexerIO() {});
        // led = new LED();

        break;

      default:
        // Replayed robot, disable IO implementations
        // (Use same number of dummy implementations as the real robot)

        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});

        hood = new Hood(new HoodIO() {}, drive::getPose, drive::getFieldVelocity);
        intakePivot = new IntakePivot(new IntakePivotIO() {});
        intakeRollers = new IntakeRollers(new IntakeRollersIO() {});
        flywheel = new Flywheel(new FlywheelIO() {}, drive::getPose, drive::getFieldVelocity);
        spindexer = new Spindexer(new SpindexerIOSim() {});
        indexer = new Indexer(new IndexerIO() {});
        // led = new LED();
        break;
    }
    // Initialize robot state
    robotState = new RobotState(this);

    // Start of Named Commands for auto:
    NamedCommands.registerCommand(
        "intakedown", robotState.seekIndefinite(IntakePivotState.DOWN).withTimeout(1));
    NamedCommands.registerCommand(
        "flywheelHoodGo",
        robotState.seekIndefinite(HoodState.SEEK_GOAL, FlywheelState.SEEK_GOAL));
    NamedCommands.registerCommand(
        "shootWhenReady",
        new SequentialCommandGroup(
            Commands.waitUntil(() -> flywheel.atGoal() && drive.atCachedAimbotHeading())
                .withTimeout(3), // hood.atGoal() &&
            robotState
                .seekIndefinite(SpindexerState.INDEXING, IndexerState.INDEXING)
                .withTimeout(3)));
    NamedCommands.registerCommand(
        "intake",
        robotState
            .seekIndefinite(IntakePivotState.DOWN, IntakeRollerState.INWARD)
            .withTimeout(2)); // figure out logic for writing time stuff
    NamedCommands.registerCommand(
        "intakeShootingPosition",
        robotState
            .seekIndefinite(IntakeRollerState.INWARD, IntakePivotState.SHOOTING_POS)
            .withTimeout(3)); // kinda jank but whatever

    NamedCommands.registerCommand(
        "autoAlignPrepareToShoot",
        new ParallelDeadlineGroup(
            DriveCommands.turnToHeadingAuto(
                    drive,
                    () -> {
                      drive.updateAimbotHeading(
                          FieldConstants.Hub.topCenterPoint.toTranslation2d());
                      return drive
                          .getCachedAimbotHeading()
                          .minus(new Rotation2d(DriveConstants.aimbotOffset));
                    })
                .withTimeout(3),
            robotState.seekIndefinite(HoodState.SEEK_GOAL, FlywheelState.SEEK_GOAL)));

    NamedCommands.registerCommand(
        "stopEverything",
        robotState
            .seek(
                HoodState.FOLD_BACK,
                FlywheelState.STOPPED,
                SpindexerState.IDLE,
                IndexerState.IDLE,
                IntakePivotState.DOWN,
                IntakeRollerState.STOPPED)
            .withTimeout(.5));

    NamedCommands.registerCommand(
        "stopIntake",
        robotState
            .seekIndefinite(IntakePivotState.DOWN, IntakeRollerState.STOPPED)
            .withTimeout(.2));

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // drive.setDefaultCommand(
    // DriveCommands.joystickDrive(
    // drive,
    // () -> -manualController.getLeftY(),
    // () -> -manualController.getLeftX(),
    // () -> -manualController.getRightX()));

    // hood.setDefaultCommand(robotState.seekIndefinite(HoodState.FOLD_BACK).repeatedly()); // comp
    // code
    hood.setDefaultCommand(robotState.seekIndefinite(HoodState.FOLD_BACK).repeatedly());
    intakeRollers.setDefaultCommand(
        robotState.seekIndefinite(IntakeRollerState.STOPPED).repeatedly());
    intakePivot.setDefaultCommand(
        robotState.seekIndefinite(IntakePivotState.DRIVING_POS).repeatedly());
    // hood.setDefaultCommand(robotState.seekIndefinite(HoodState.MANUAL).repeatedly());
    flywheel.setDefaultCommand(robotState.seekIndefinite(FlywheelState.STOPPED).repeatedly());

    /* COMP CONTROLS */
    // aimbot trigger
    Trigger aimbotHeld = controller.rightTrigger();

    // robotState
    //     .getTrenchWarningTrigger()
    //     .and(aimbotHeld.negate())
    //     .whileTrue(robotState.seekIndefinite(HoodState.FOLD_BACK));

    // robotState
    //     .getTrenchHardTrigger()
    //     .and(
    //         new Trigger(
    //             () ->
    //                 hood.getMeasuredAngleRad()
    //                     > FieldConstants.TrenchSafetyConstants.HOOD_SAFE_ANGLE_RAD))
    //     .onTrue(Commands.runOnce(() -> drive.setTrenchProtection(true)))
    //     .onFalse(Commands.runOnce(() -> drive.setTrenchProtection(false)));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when Start button is pressed
    controller
        .start()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    // set intake pivot down when left bumper held
    controller
        .leftTrigger()
        .whileTrue(robotState.seekIndefinite(IntakePivotState.DOWN, IntakeRollerState.INWARD));
    controller.leftBumper().whileTrue(robotState.seekIndefinite(IntakePivotState.UP));
    controller.a().whileTrue(robotState.seekIndefinite(IntakeRollerState.OUTWARD));

    // counter indexing
    controller
        .y()
        .whileTrue(robotState.seekIndefinite(IndexerState.REVERSE, SpindexerState.REVERSE))
        .onFalse(robotState.seek(IndexerState.IDLE, SpindexerState.IDLE));

    // aimbot at target while shooting
    aimbotHeld
        .whileTrue(
            new ParallelCommandGroup(
                DriveCommands.joystickDriveAtAngle(
                    drive,
                    () -> -controller.getLeftY() * 0.55,
                    () -> -controller.getLeftX() * 0.55,
                    () -> {
                      drive.updateAimbotHeading(
                          FieldConstants.Hub.topCenterPoint.toTranslation2d());
                      return drive
                          .getCachedAimbotHeading()
                          .minus(new Rotation2d(DriveConstants.aimbotOffset));
                    }),
                robotState.seekIndefinite(FlywheelState.SEEK_GOAL, HoodState.SEEK_GOAL),
                // feed when flywheel ready
                new SequentialCommandGroup(
                    Commands.waitUntil(
                        () -> hood.atGoal() && drive.atCachedAimbotHeading() && flywheel.atGoal()),
                    robotState.seekIndefinite(
                        SpindexerState.INDEXING,
                        IndexerState.INDEXING,
                        IntakeRollerState.INWARD,
                        IntakePivotState.DOWN))))
        .onFalse(
            robotState.seek(
                SpindexerState.IDLE,
                IndexerState.IDLE,
                IntakePivotState.DRIVING_POS,
                FlywheelState.STOPPED));

    // pass to target
    controller
        .rightBumper()
        .whileTrue(
            new ParallelCommandGroup(
                DriveCommands.joystickDriveAtAngle(
                    drive,
                    () -> -controller.getLeftY() * 0.8,
                    () -> -controller.getLeftX() * 0.8,
                    () -> {
                      drive.updateAimbotHeading(drive.getBestPassingTarget());
                      Logger.recordOutput("test/targetPose", drive.getBestPassingTarget()); // debug
                      return drive
                          .getCachedAimbotHeading()
                          .minus(new Rotation2d(DriveConstants.aimbotOffset));
                    }),
                robotState.seekIndefinite(FlywheelState.PASS_BALL, HoodState.PASS_BALL),
                // feed when flywheel ready
                new SequentialCommandGroup(
                    Commands.waitUntil(
                        () ->
                            hood.atGoal()
                                && drive.atCachedAimbotHeadingForPassing()
                                && flywheel.atGoal()),
                    robotState.seekIndefinite(
                        SpindexerState.INDEXING,
                        IndexerState.INDEXING,
                        IntakeRollerState.INWARD,
                        IntakePivotState.SHOOTING_POS))))
        .onFalse(
            robotState.seek(
                SpindexerState.IDLE,
                IndexerState.IDLE,
                IntakePivotState.DRIVING_POS,
                FlywheelState.STOPPED));

    /* CONTROLLER 2 FOR TESTING */
    // intake testing ; should be the same as the other
    manualController
        .leftTrigger()
        .whileTrue(robotState.seekIndefinite(IntakeRollerState.INWARD))
        .onFalse(robotState.seekIndefinite(IntakeRollerState.STOPPED));

    // change the state of the intake pivot
    manualController.povDown().whileTrue(robotState.seekIndefinite(IntakePivotState.DOWN));
    manualController.povUp().whileTrue(robotState.seekIndefinite(IntakePivotState.UP));

    // try this when testing shooting; if it doesn't work go back to jank
    manualController
        .rightTrigger()
        .whileTrue(
            new ParallelCommandGroup(
                DriveCommands.joystickDriveAtAngle(
                    drive,
                    () -> -manualController.getLeftY() * 0.55,
                    () -> -manualController.getLeftX() * 0.55,
                    () -> {
                      drive.updateAimbotHeading(
                          FieldConstants.Hub.topCenterPoint.toTranslation2d());
                      return drive.getCachedAimbotHeading();
                    }),
                flywheel.runVelocityCommand(tuneFlywheelRPM::get),
                hood.moveToAngle(tuneHoodDeg::get),
                new SequentialCommandGroup(
                    Commands.waitUntil(
                        // () -> hood.atGoal() &&
                        () -> drive.atCachedAimbotHeading() && flywheel.atGoal()),
                    robotState.seekIndefinite(
                        SpindexerState.INDEXING,
                        IndexerState.INDEXING,
                        IntakeRollerState.INWARD,
                        IntakePivotState.SHOOTING_POS))))
        .onFalse(
            robotState.seekIndefinite(
                SpindexerState.IDLE,
                IndexerState.IDLE,
                IntakePivotState.DOWN,
                IntakeRollerState.INWARD));

    manualController
        .b()
        .whileTrue(
            new ParallelCommandGroup(
                flywheel.runVelocityCommandRPM(tuneFlywheelRPM::get),
                hood.moveToAngle(tuneHoodDeg::get),
                new SequentialCommandGroup(
                    Commands.waitUntil(() -> hood.atGoal() && flywheel.atGoal()),
                    robotState.seekIndefinite(SpindexerState.INDEXING, IndexerState.INDEXING))))
        .onFalse(robotState.seekIndefinite(SpindexerState.IDLE, IndexerState.IDLE));

    manualController
        .x()
        .whileTrue(
            new ParallelCommandGroup(
                robotState.seekIndefinite(
                    HoodState.SEEK_GOAL, FlywheelState.SEEK_GOAL, IntakeRollerState.INWARD),
                new SequentialCommandGroup(
                    Commands.waitUntil(() -> hood.atGoal() && flywheel.atGoal()),
                    robotState.seekIndefinite(SpindexerState.INDEXING, IndexerState.INDEXING))))
        .onFalse(
            robotState
                .seekIndefinite(SpindexerState.IDLE, IndexerState.IDLE, IntakeRollerState.STOPPED)
                .alongWith(robotState.seek(HoodState.FOLD_BACK, FlywheelState.STOPPED)));

    manualController
        .y()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> manualController.getLeftY() * 0.55,
                () -> manualController.getLeftX() * 0.55,
                () -> {
                  drive.updateAimbotHeading(FieldConstants.Hub.topCenterPoint.toTranslation2d());
                  return (drive
                      .getCachedAimbotHeading()
                      .minus(new Rotation2d(DriveConstants.aimbotOffset)));
                }));

    // for debugging hood + flywheels
    manualController
        .povRight()
        .whileTrue(flywheel.runVelocityCommandRPM(tuneFlywheelRPM::get))
        .onFalse(flywheel.stopCommand());
    manualController.povLeft().onTrue(robotState.seek(IntakePivotState.SHOOTING_POS));

    // zero hood angle please do this before updating code (limit switch doesn't
    // work)
    manualController.a().onTrue(hood.moveToAngle(() -> 10));

    // zero the gyro
    manualController
        .povLeft()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
