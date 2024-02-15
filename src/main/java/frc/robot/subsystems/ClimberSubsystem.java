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
    private double currentClimberPosition;
    
    public ClimberSubsystem(){
        climbMotor1 = new CANSparkMax(Constants.Climber.climberMotorID1, MotorType.kBrushless);
        climbMotor2 = new CANSparkMax(Constants.Climber.climberMotorID2, MotorType.kBrushless);
        currentClimberPosition = 0;
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
        //CurrentManager.updateCurrent(1, CurrentManager.Subsystem.Climber);
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
