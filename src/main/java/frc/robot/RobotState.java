package frc.robot;

public class RobotState {
    public enum IntakePivotState {
        UP,
        DOWN
    }


    public enum IntakeRollerState {
        INWARD,
        OUTWARD,
        STOPPED
    }

    public enum ShooterState {
        SEEK_GOAL,
        PASS_BALL,
        STOPPED
    }

    public enum HoodState {
        SEEK_GOAL,
        PASS_BALL,
        FOLD_BACK
    }


}
