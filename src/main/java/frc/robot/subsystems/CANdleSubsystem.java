package frc.robot.subsystems;
import java.util.Optional;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;

import edu.wpi.first.units.Current;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;


public class CANdleSubsystem extends SubsystemBase implements Component{
    
    private final CANdle m_candle = new CANdle(Constants.CANdle.CANdleID, Constants.CANivoreID);

    public enum AnimationTypes {
        Shoot(new ColorFlowAnimation(128, 20, 70, 0, 0.7, Constants.CANdle.LEDCount, Direction.Forward)),
        Auton(new SingleFadeAnimation(50, 2, 200, 0, 0.5, Constants.CANdle.LEDCount)),
        TeleopMovement(new SingleFadeAnimation(50, 2, 200, 0, 0.5, Constants.CANdle.LEDCount)),
        InTake(new SingleFadeAnimation(50, 2, 200, 0, 0.5, Constants.CANdle.LEDCount)),
        Climb(new SingleFadeAnimation(50, 2, 200, 0, 0.5, Constants.CANdle.LEDCount)),
        Dance(new SingleFadeAnimation(50, 2, 200, 0, 0.5, Constants.CANdle.LEDCount)),
        Idle(null);

        private Animation animation;

        private AnimationTypes(Animation animation){
            this.animation = animation;
        }
        public Animation getAnimation(){
            return animation;
        }
    }

    private AnimationTypes currentAnimation;
    private AnimationTypes lastAnimation;
 
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
    }

    public AnimationTypes getAnimation(){
        return currentAnimation;
    }
    

    @Override
    public void periodic() {
        if (currentAnimation != lastAnimation){
            lastAnimation = currentAnimation;

            if(currentAnimation.getAnimation() != null) {
                m_candle.animate(currentAnimation.getAnimation());
            }else{
                Optional<Alliance> alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    if (alliance.get() == Alliance.Red) {
                        m_candle.setLEDs(255, 56, 56);
                    }
                    if (alliance.get() == Alliance.Blue) {
                        m_candle.setLEDs(41, 118, 242);
                    }
                }else{
                    m_candle.setLEDs(255, 255, 255);
                }
                
            }
            SmartDashboard.putString("Current Robot LED Animation", CurrentManager.isOverNominal() ? "Disabled due to over current" : currentAnimation.name());
            SmartDashboard.putNumber("CANdle current draw" , m_candle.getCurrent());
        }
    }
    @Override
    public double getRequestedCurrent(){
        return 0;
    }
    @Override
    public void allocateCurrent(double current){
        //set motor controller current
    }
    @Override
    public int getPriority(){
        return 6;
    }
    @Override
    public void updateBasedOnAllocatedCurrent(){
        //update motor controller based on allocated current
    }
}
