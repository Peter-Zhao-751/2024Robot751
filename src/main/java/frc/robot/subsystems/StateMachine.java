package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.Aimbot;
import frc.robot.commands.lowLevelCommands.CANdleController;

public class StateMachine {

    public enum State {
        Idle,
        TeleopDrive,
        Auton,
        Aimbot,
        Shooter,
        Intake,
        Disabled,
        Climb,
    }

    private StateMachine(){
        // This is a utility class and should not be instantiated.
    }

    public static State state = State.Idle;

    public static void setState(State newState) {
        state = newState;
    }
    public static State getState() {
        return state;
    }

    public static void update() {
        switch (state) {
            case Idle:
                SmartDashboard.putString("Robot State", "Idle");
                CANdleController.changeAnimation(CANdleSubsystem.AnimationTypes.Idle);
                break;
            case TeleopDrive:
                SmartDashboard.putString("Robot State", "Teloep Drive");
                CANdleController.changeAnimation(CANdleSubsystem.AnimationTypes.TeleopMovement);
                break;
            case Auton:
                SmartDashboard.putString("Robot State", "Auton");
                CANdleController.changeAnimation(CANdleSubsystem.AnimationTypes.Auton);
                break;
            case Aimbot:
                SmartDashboard.putString("Robot State", "Aimbot");
                CANdleController.changeAnimation(CANdleSubsystem.AnimationTypes.Aimbot);
                break;
            case Shooter:
                SmartDashboard.putString("Robot State", "Shoot");
                CANdleController.changeAnimation(CANdleSubsystem.AnimationTypes.Shoot);
                break;
            case Intake:
                SmartDashboard.putString("Robot State", "Intake");
                CANdleController.changeAnimation(CANdleSubsystem.AnimationTypes.Intake);
                break;
            case Disabled:
                SmartDashboard.putString("Robot State", "Disabled");
                CANdleController.changeAnimation(CANdleSubsystem.AnimationTypes.Disabled);
                break;
            case Climb:
                SmartDashboard.putString("Robot State", "Climb");
                CANdleController.changeAnimation(CANdleSubsystem.AnimationTypes.Climb);
                break;
        }
    }
}