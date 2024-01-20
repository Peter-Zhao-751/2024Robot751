package frc.robot.subsystems;
import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;

import edu.wpi.first.units.Current;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CANdleSubsystem extends SubsystemBase{
    
    private final CANdle m_candle = new CANdle(Constants.CANdle.CANdleID, "rio");
    private final int LEDCount = Constants.CANdle.LEDCount;
    public enum AnimationTypes {
        Shoot,
        Auton,
        TeleopMovement,
        InTake,
        Climb,
        Dance,
        Idle
    }
    private AnimationTypes currentAnimation;
    private Animation animationToDisplay = null;
 
    public CANdleSubsystem() {

        changeAnimation(AnimationTypes.Idle);
        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = true;
        configAll.disableWhenLOS = false;
        configAll.stripType = LEDStripType.GRB;
        configAll.brightnessScalar = 0.1;
        configAll.vBatOutputMode = VBatOutputMode.Modulated;
        m_candle.configAllSettings(configAll, 100);
    }

    public void changeAnimation(AnimationTypes newAnimation){
        currentAnimation = newAnimation;
        switch (newAnimation){
            case Shoot: 
                animationToDisplay = new ColorFlowAnimation(128, 20, 70, 0, 0.7, LEDCount, Direction.Forward);
            default:
                animationToDisplay = new SingleFadeAnimation(50, 2, 200, 0, 0.5, LEDCount);
        }
    }

    @Override
    public void periodic() {
        if(animationToDisplay != null) {
            m_candle.animate(animationToDisplay);
        }
        SmartDashboard.setDefaultString("Current Robot LED Animation", CurrentManager.isOverNominal() ? currentAnimation.name() : "Disabled due to over current");
    }
}
