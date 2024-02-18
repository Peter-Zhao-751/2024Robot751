package frc.robot.subsystems;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ClimberSubsystem extends SubsystemBase implements Component{
    private CANSparkMax climbMotor1;
    private CANSparkMax climbMotor2;
    private KalmanFilter.States currentEncoderStates;
    
    public ClimberSubsystem(){
        climbMotor1 = new CANSparkMax(Constants.Climber.leftClimberMotorID, MotorType.kBrushless);
        climbMotor2 = new CANSparkMax(Constants.Climber.rightClimberMotorID, MotorType.kBrushless);
        currentEncoderStates = new KalmanFilter.States(0);
    }
    public void climb(double speed){
        climbMotor1.set(speed);
        climbMotor2.set(speed);
    }
    public void stop(){
        climbMotor1.set(0);
        climbMotor2.set(0);
    }
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Total Climber Current Draw", climbMotor1.getOutputCurrent() + climbMotor2.getOutputCurrent());
        climbMotor1.getEncoder().getVelocity();

        //TODO: update encoder states with velocity
        //CurrentManager.updateCurrent(1, CurrentManager.Subsystem.Climber);
    }
    private double getCurrentPosition(){
        //basically a solved differential equation for the extension of the spring as a function of how many rotations it has done. r = (k-TR)  c = 2pi(k-TR)                                          actually d = (r sub0 + T * R)^2, where r sub 0 is the max radius and 



        return climbMotor1.getEncoder().getPosition();
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
