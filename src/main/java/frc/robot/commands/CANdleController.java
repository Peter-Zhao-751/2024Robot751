package frc.robot.commands;

import frc.robot.subsystems.CANdleSubsystem;

public class CANdleController {
    public static final CANdleSubsystem m_candle = new CANdleSubsystem();

    public static CANdleSubsystem.AnimationTypes currentAnimation(){
        return m_candle.getAnimation();
    }
    public static void changeAnimation(CANdleSubsystem.AnimationTypes animation){
        m_candle.changeAnimation(animation);
    }
}