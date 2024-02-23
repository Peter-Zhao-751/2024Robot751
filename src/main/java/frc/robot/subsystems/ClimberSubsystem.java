package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ClimberSubsystem extends SubsystemBase implements Component{
    private final CANSparkMax leftClimberMotor;
    private final CANSparkMax rightClimberMotor;
    private final KalmanFilter.States currentEncoderStates;
    
    public ClimberSubsystem(){
        leftClimberMotor = new CANSparkMax(Constants.Climber.leftClimberMotorID, MotorType.kBrushless);
        rightClimberMotor = new CANSparkMax(Constants.Climber.rightClimberMotorID, MotorType.kBrushless);
        currentEncoderStates = new KalmanFilter.States(0);
    }

    public void rightMotor(double speed){
        rightClimberMotor.set(speed);
    }

    public void leftMotor(double speed){
        leftClimberMotor.set(speed);
    }

    public void climb(double speed){
        leftClimberMotor.set(speed);
        rightClimberMotor.set(speed);
    }

    public void retract(double speed){
        leftClimberMotor.set(-speed);
        rightClimberMotor.set(-speed);
    }

    public void stop(){
        leftClimberMotor.set(0);
        rightClimberMotor.set(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Total Climber Current Draw", leftClimberMotor.getOutputCurrent() + rightClimberMotor.getOutputCurrent());
        leftClimberMotor.getEncoder().getVelocity();

        //TODO: update encoder states with velocity
        //CurrentManager.updateCurrent(1, CurrentManager.Subsystem.Climber);
    }

    private double getCurrentPosition(){
        //basically a solved differential equation for the extension of the spring as a function of how many rotations it has done. r = (k-TR)  c = 2pi(k-TR)                                          actually d = (r sub0 + T * R)^2, where r sub 0 is the max radius and 



        return leftClimberMotor.getEncoder().getPosition();
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
        return 5;
    }

    @Override
    public void updateBasedOnAllocatedCurrent(){
        //update motor controller based on allocated current
    }
}
