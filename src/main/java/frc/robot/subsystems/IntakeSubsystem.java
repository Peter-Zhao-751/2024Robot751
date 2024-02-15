package frc.robot.subsystems;

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
    
    public IntakeSubsystem(){
        swivelMotor1 = new CANSparkMax(Constants.Intake.swivelMotorID1, MotorType.kBrushless);
        swivelMotor2 = new CANSparkMax(Constants.Intake.swivelMotorID2, MotorType.kBrushless);
        intakeMotor = new TalonFX(Constants.Intake.intakeMotorID);
        transferMotor = new CANSparkMax(Constants.Intake.transportMotorID, MotorType.kBrushless);
    }
    public void swivel(double speed){
        swivelMotor1.set(speed);
        swivelMotor2.set(speed);
    }
    public void transfer(double speed){
        transferMotor.set(speed);
    }
    public void intake(double speed){
        intakeMotor.set(speed);
    }
    public void stop(){
        swivelMotor1.set(0);
        swivelMotor2.set(0);
        transferMotor.set(0);
        intakeMotor.set(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Total Intake Current Draw", swivelMotor1.getOutputCurrent() + swivelMotor2.getOutputCurrent() + transferMotor.getOutputCurrent() + intakeMotor.getSupplyCurrent().getValue());
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
