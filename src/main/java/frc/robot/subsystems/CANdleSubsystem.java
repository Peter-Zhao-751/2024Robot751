package frc.robot.subsystems;
import java.util.Optional;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utility.CurrentManager;
import frc.robot.utility.TelemetryUpdater;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;


public class CANdleSubsystem extends SubsystemBase implements Component{
    
    private final CANdle m_candle = new CANdle(Constants.CANdle.CANdleID, Constants.CANivoreID);

    public enum AnimationTypes {
        Shoot(new ColorFlowAnimation(128, 20, 70, 0, 0.7, Constants.CANdle.LEDCount, Direction.Forward, 8)),
        Auton(new SingleFadeAnimation(50, 2, 200, 0, 0.5, Constants.CANdle.LEDCount, 8)),
        TeleopMovement(new SingleFadeAnimation(50, 2, 200, 0, 0.5, Constants.CANdle.LEDCount, 8)),
        Intake(new SingleFadeAnimation(50, 2, 200, 0, 0.5, Constants.CANdle.LEDCount, 8)),
        Climb(new SingleFadeAnimation(50, 2, 200, 0, 0.5, Constants.CANdle.LEDCount, 8)),
        Dance(new SingleFadeAnimation(50, 2, 200, 0, 0.5, Constants.CANdle.LEDCount, 8), 2000),
        Alert(new TwinkleAnimation(255, 255, 255, 255, 0.5, Constants.CANdle.LEDCount, TwinkleAnimation.TwinklePercent.Percent100, 8)),
        Aimbot(new TwinkleAnimation(0, 0, 255, 0, 0.5, Constants.CANdle.LEDCount, TwinkleAnimation.TwinklePercent.Percent100, 8)),
        Idle(null),
        Disabled(null);

        private Animation animation;
        private double decay; // in milliseconds
        private double startAnimationTime;

        private AnimationTypes(Animation animation){
            this(animation, -1);
        }
        private AnimationTypes(Animation animation, double decay) {
            this.animation = animation;
            this.decay = decay;
            this.startAnimationTime = 0;
        }
        public Animation getAnimation(){
            return animation;
        }
        public double getDecay(){
            return decay;
        }
        public void setStartAnimationTime(){
            startAnimationTime = System.currentTimeMillis();
        }
        public double getStartAnimationTime(){
            return startAnimationTime;
        }
        public boolean isDone(){
            if (decay > 0){
                return (System.currentTimeMillis() - startAnimationTime) > decay;
            }else{
                return false;
            }
        }
    }

    private AnimationTypes desiredAnimation;
    private AnimationTypes currentAnimation;
    private AnimationTypes lastAnimation;

    private double allocatedCurrent;
 
    public CANdleSubsystem() {
        currentAnimation = AnimationTypes.Idle;
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
        desiredAnimation = newAnimation;
        if (currentAnimation.decay < 0) lastAnimation = currentAnimation;
    }

    public AnimationTypes getAnimation(){
        return currentAnimation;
    }

    public void twinkle(){
        changeAnimation(AnimationTypes.Alert);
    }

    private void setColorToAllianceColor(){
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
            System.err.println("CANdle error, Alliance not found (not epic gamer moment)");
        }
    }
    @Override
    public void periodic() {
        if (desiredAnimation != currentAnimation){
            desiredAnimation.setStartAnimationTime();
            
            if (desiredAnimation.getAnimation() != null) m_candle.animate(desiredAnimation.getAnimation());
            else setColorToAllianceColor();

            currentAnimation = desiredAnimation;

            TelemetryUpdater.setTelemetryValue("Current Robot LED Animation", CurrentManager.isOverNominal() ? "Disabled due to over-current" : currentAnimation.name());
        }else if (currentAnimation.isDone()){
            changeAnimation(lastAnimation);
            currentAnimation = lastAnimation;
            desiredAnimation = lastAnimation;
        }
    }

    @Override
    public double getCurrentDraw(){
        return m_candle.getCurrent();
    }
    @Override
    public void allocateCurrent(double current){
        //set motor controller current
    }
    @Override
    public int getPriority(){
        return 6;
    }
}
