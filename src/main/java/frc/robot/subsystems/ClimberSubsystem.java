package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ClimberSubsystem extends SubsystemBase implements Component{
    private final CANSparkMax leftClimberMotor;
    private final CANSparkMax rightClimberMotor;

    private double allocatedCurrent;
    
    public ClimberSubsystem(){
        leftClimberMotor = new CANSparkMax(Constants.Climber.leftClimberMotorID, MotorType.kBrushless);
        rightClimberMotor = new CANSparkMax(Constants.Climber.rightClimberMotorID, MotorType.kBrushless);
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
        SmartDashboard.putNumber("Climber Current Draw", getCurrentDraw());
        leftClimberMotor.getEncoder().getVelocity();
    }

    private double getCurrentPosition(){

        return leftClimberMotor.getEncoder().getPosition();
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
