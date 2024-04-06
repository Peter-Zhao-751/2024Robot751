package frc.robot.commands.gamepieceCommands;

import frc.robot.subsystems.CANdleSubsystem;

public class CANdleControllerCommand {
    private static CANdleSubsystem m_Candle = null;
    private CANdleControllerCommand() {
        throw new UnsupportedOperationException("CANdleController class cannot be instantiated");
    }
    public static void setCandle(CANdleSubsystem candle){
        m_Candle = candle;
    }

    public static CANdleSubsystem.AnimationTypes currentAnimation(){
        return m_Candle.getAnimation();
    }
    public static void changeAnimation(CANdleSubsystem.AnimationTypes animation){
        m_Candle.changeAnimation(animation);
    }
}