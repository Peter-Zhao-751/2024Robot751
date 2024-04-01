package frc.robot.utility;

import frc.robot.commands.gamepieceCommands.CANdleController;
import frc.robot.subsystems.CANdleSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class StateMachine {

    public enum State {
        Idle("Idle", CANdleSubsystem.AnimationTypes.Idle),
        TeleopDrive("Teleop Drive", CANdleSubsystem.AnimationTypes.TeleopMovement),
        Auton("Auton", CANdleSubsystem.AnimationTypes.Auton),
        Aimbot("Aimbot", CANdleSubsystem.AnimationTypes.Aimbot),
        Shoot("Shoot", CANdleSubsystem.AnimationTypes.Shoot),
        Intake("Intake", CANdleSubsystem.AnimationTypes.Intake),
        Disabled("Disabled", CANdleSubsystem.AnimationTypes.Disabled),
        Climb("Climb", CANdleSubsystem.AnimationTypes.Climb);

        public final String stateName;
        public final CANdleSubsystem.AnimationTypes animation;

        State(String stateName, CANdleSubsystem.AnimationTypes animation) {
            this.stateName = stateName;
            this.animation = animation;
        }
    }

    public static State state = State.Idle;

    private StateMachine(){
    }

    public static void setState(State newState) {
        state = newState;
    }

    public static State getState() {
        return state;
    }

    public static void update() {
		CANdleController.changeAnimation(state.animation);
		if (state == State.Auton || state == State.Aimbot) {
            LimelightSubsystem.getInstance().setLEDMode(LimelightSubsystem.LEDMode.ON);
        } else {
            LimelightSubsystem.getInstance().setLEDMode(LimelightSubsystem.LEDMode.OFF);
        }

    }

    public static void periodic() {
        TelemetryUpdater.setTelemetryValue("Robot State", state.stateName);
    }

    public static boolean isPerformingAction(){
        return state == State.Aimbot || state == State.Intake || state == State.Shoot || state == State.Climb;
    }
}