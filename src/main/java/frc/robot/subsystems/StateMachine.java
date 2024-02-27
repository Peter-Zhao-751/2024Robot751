package frc.robot.subsystems;

import frc.robot.commands.AimBot;

public class StateMachine {

    public enum State {
        IDLE,
        TELEOP_DRIVE,
        AUTON,
        AIMBOT,
        SHOOT,
        INTAKE,
        DISABLED,
        CLIMB,
    }

    private StateMachine(){
        // This is a utility class and should not be instantiated.
    }

    public static State state = State.IDLE;

    public static void setState(State newState) {
        state = newState;
    }
    
}