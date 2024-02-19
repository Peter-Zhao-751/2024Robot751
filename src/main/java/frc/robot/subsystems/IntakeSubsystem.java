package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Current;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class IntakeSubsystem extends SubsystemBase implements Component{
    private CANSparkMax swivelMotor1;
    private CANSparkMax swivelMotor2;
    private CANSparkMax transferMotor;
    private TalonFX intakeMotor;

    private PIDController swivelPIDController;
    
    public IntakeSubsystem(){
        swivelMotor1 = new CANSparkMax(Constants.Intake.leftSwivelMotorID, MotorType.kBrushless);
        swivelMotor2 = new CANSparkMax(Constants.Intake.rightSwivelMotorID, MotorType.kBrushless);
        intakeMotor = new TalonFX(Constants.Intake.intakeMotorID);
        transferMotor = new CANSparkMax(Constants.Intake.transportMotorID, MotorType.kBrushless);

        swivelPIDController = new PIDController(Constants.Intake.kPSwivelController, Constants.Intake.kISwivelController, Constants.Intake.kDSwivelController);
    }
    
    public void setSwivelSpeed(double speed){
        swivelMotor1.set(speed);
        swivelMotor2.set(speed);

    }
    public void setTransferSpeed(double speed){
        transferMotor.set(speed);
    }
    public void setIntakeSpeed(double speed){
        intakeMotor.set(speed);
    }
    public void stopAll(){
        swivelMotor1.set(0);
        swivelMotor2.set(0);
        transferMotor.set(0);
        intakeMotor.set(0);
    }
    public double getSwivelPosition(){
        return swivelMotor1.getEncoder().getPosition();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Total Intake Current Draw", swivelMotor1.getOutputCurrent() + swivelMotor2.getOutputCurrent() + transferMotor.getOutputCurrent() + intakeMotor.getSupplyCurrent().getValue());
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
