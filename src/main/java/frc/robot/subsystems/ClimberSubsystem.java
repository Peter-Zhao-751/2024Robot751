package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utility.TelemetryUpdater;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;

// not done yet dont kill me
public class ClimberSubsystem extends SubsystemBase implements Component {
    private static ClimberSubsystem instance;

    private final CANSparkMax leftClimberMotor;
    private final CANSparkMax rightClimberMotor;

    private final RelativeEncoder leftClimberEncoder;
    private final RelativeEncoder rightClimberEncoder;

    private final PIDController climberPIDController;

    private double leftDesiredLocation;
    private double rightDesiredLocation;

    private double allocatedCurrent;

    public static ClimberSubsystem getInstance(){
        if(instance == null) instance = new ClimberSubsystem();
        return instance;
    }
    
    private ClimberSubsystem(){
        leftClimberMotor = new CANSparkMax(Constants.Climber.leftClimberMotorID, MotorType.kBrushless);
        rightClimberMotor = new CANSparkMax(Constants.Climber.rightClimberMotorID, MotorType.kBrushless);

        leftClimberEncoder = leftClimberMotor.getEncoder();
        rightClimberEncoder = rightClimberMotor.getEncoder();

        //leftClimberMotor.setInverted(true);
        leftClimberMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        rightClimberMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);

        climberPIDController = new PIDController(Constants.Climber.kPClimbController, Constants.Climber.kIClimbController, Constants.Climber.kDClimbController);

        leftDesiredLocation = 0;
        rightDesiredLocation = 0;
    }

    private double getLeftPosition(){ 
        return leftClimberEncoder.getPosition() / Constants.Climber.kGearRatio * 2 * Constants.Climber.kSpoolRadius * Math.PI;
    }

    private double getRightPosition(){
        return rightClimberEncoder.getPosition() / Constants.Climber.kGearRatio * 2 * Constants.Climber.kSpoolRadius * Math.PI;
    }

    /**
     * Set the speed of the left climber motor
     * @param location in distance
     */
    public void setLeftDesiredLocation(double location){
        leftDesiredLocation = location;
    }

    public void setLeftClimberMotor(double speed){
        leftDesiredLocation += speed;
    }

    /**
     * Set the speed of the right climber motor
     * @param location in distance
     */

    public void setRightDesiredLocation(double location){
        rightDesiredLocation = location;
    }

    public void setRightClimberMotor(double speed){
        rightDesiredLocation += speed;
    }

    /**
     * Stop both climber motors
     */
    public void stop(){
        leftClimberMotor.set(0);
        rightClimberMotor.set(0);
    }

    @Override
    public void periodic() {

        double leftOutput = climberPIDController.calculate(getLeftPosition(), leftDesiredLocation);
        double rightOutput = climberPIDController.calculate(getRightPosition(), rightDesiredLocation);

        if (leftOutput > 0 && getLeftPosition() > Constants.Climber.maxClimberHeight) leftOutput = 0;
        if (rightOutput > 0 && getRightPosition() > Constants.Climber.maxClimberHeight) rightOutput = 0;
        if (leftOutput < 0 && getLeftPosition() < Constants.Climber.minClimberHeight) leftOutput = 0;
        if (rightOutput < 0 && getRightPosition() < Constants.Climber.minClimberHeight) rightOutput = 0;

        leftClimberMotor.set(leftOutput);
        rightClimberMotor.set(rightOutput);

        /*TelemetryUpdater.setTelemetryValue("Climber Current Draw", getCurrentDraw());
        TelemetryUpdater.setTelemetryValue("Left Climber Position", getLeftPosition());
        TelemetryUpdater.setTelemetryValue("Right Climber Position", getRightPosition());*/
    }

    @Override
    public double getCurrentDraw(){
        return leftClimberMotor.getOutputCurrent() + rightClimberMotor.getOutputCurrent();
    }

    @Override
    public void allocateCurrent(double current){
        //set motor controller current
    }

    @Override
    public int getPriority(){
        return 5;
    }
}
