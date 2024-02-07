package frc.robot.commands;

import com.ctre.phoenix.led.CANdle;

import frc.robot.subsystems.CANdleSubsystem;

public class CANdleController {
    public static final CANdleSubsystem m_candle = new CANdleSubsystem();
    
    public static void changeAnimation(CANdleSubsystem.AnimationTypes animation){
        m_candle.changeAnimation(animation);
    }
}
