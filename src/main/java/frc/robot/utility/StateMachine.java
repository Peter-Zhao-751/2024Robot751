package frc.robot.utility;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.AimBot;
import frc.robot.commands.lowLevelCommands.CANdleController;
import frc.robot.subsystems.CANdleSubsystem;
import frc.robot.subsystems.CANdleSubsystem.AnimationTypes;

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

        public String stateName;
        public CANdleSubsystem.AnimationTypes animation;

        private State(String stateName, CANdleSubsystem.AnimationTypes animation) {
            this.stateName = stateName;
            this.animation = animation;
        }
    }

    private StateMachine(){
        // This is a utility class and should not be instantiated.
        throw new UnsupportedOperationException("This is a utility class and should not be instantiated.");
    }

    private static State state = State.Idle;

    public static void setState(State newState) {
        state = newState;
    }
    
    public static State getState() {
        return state;
    }

    public static void update() {
        SmartDashboard.putString("Robot State", state.stateName);
        CANdleController.changeAnimation(state.animation);
    }
}