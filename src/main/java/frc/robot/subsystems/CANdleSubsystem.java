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
    
    private final CANdle m_candle = new CANdle(Constants.CANdle.CANdleID, Constants.CANivoreID);
    private final int LEDCount = Constants.CANdle.LEDCount;
    public enum Modules {

        SwerveBL (false, 0),
        SwerveBR (false, 1),
        SwerveFL (false, 2),
        SwerveFR (false, 3),
        Shooter (false, 4),
        Intake (false, 5),
        Climber (false, 6),
        Debug (false, 7);

        private boolean initialized;
        private int LEDIndex;
        private Modules (boolean initialized, int LEDIndex){
            this.initialized = initialized;
            this.LEDIndex = LEDIndex;
        }
        public boolean isInitialized() {
            return initialized;
        }
        public int getLEDIndex() {
            return LEDIndex;
        }
        public void setInitialized(boolean initialized) {
            this.initialized = initialized;
        }
    }
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
        
        SmartDashboard.setDefaultString("Current Robot LED Animation", CurrentManager.isOverNominal() ? "Disabled due to over current" : currentAnimation.name());
    }

    public void moduleLED(Modules module) {
        if(module.isInitialized()) m_candle.setLEDs(0, 255, 0, 0, module.getLEDIndex(), 1);
        else m_candle.setLEDs(255, 0, 0, 0, module.getLEDIndex(), 1);
    }

    public void onboardLED(Modules module) {
        module.setInitialized(!module.isInitialized());
        moduleLED(module);
    }
}
