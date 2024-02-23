package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Current;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class IntakeSubsystem extends SubsystemBase implements Component {
    private final CANSparkMax swivelMotor1;
    private final CANSparkMax swivelMotor2;
    private final TalonFX intakeMotor;
    private final DutyCycleEncoder angleEncoder;

    private final PIDController swivelPIDController;
    
    public IntakeSubsystem(){
        swivelMotor1 = new CANSparkMax(Constants.Intake.leftSwivelMotorID, MotorType.kBrushless);
        swivelMotor2 = new CANSparkMax(Constants.Intake.rightSwivelMotorID, MotorType.kBrushless);

        swivelMotor1.setIdleMode(CANSparkMax.IdleMode.kBrake);
        swivelMotor2.setIdleMode(CANSparkMax.IdleMode.kBrake);

        swivelMotor1.follow(swivelMotor1);

        angleEncoder = new DutyCycleEncoder(Constants.Intake.encoderID);

        intakeMotor = new TalonFX(Constants.Intake.intakeMotorID);

        swivelPIDController = new PIDController(Constants.Intake.kPSwivelController, Constants.Intake.kISwivelController, Constants.Intake.kDSwivelController);
    } 
    
    public void setSwivelSpeed(double speed){
        swivelMotor1.set(speed);
    }
    
    public void setIntakeSpeed(double speed){
        intakeMotor.set(speed);
    }

    public void stopAll(){
        swivelMotor1.set(0);
        intakeMotor.set(0);
    }
    public double getSwivelPosition(){
        return angleEncoder.getAbsolutePosition();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Total Intake Current Draw", swivelMotor1.getOutputCurrent() + swivelMotor2.getOutputCurrent() + intakeMotor.getSupplyCurrent().getValue());
        SmartDashboard.putNumber("Intake Swivel Position", getSwivelPosition());
        
        //CurrentManager.updateCurrent(1, CurrentManager.Subsystem.Intake);
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
        return 2;
    }
    @Override
    public void updateBasedOnAllocatedCurrent(){
        //update motor controller based on allocated current
    }
}
