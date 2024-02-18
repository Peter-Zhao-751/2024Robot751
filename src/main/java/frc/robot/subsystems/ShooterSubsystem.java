package frc.robot.subsystems;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ShooterSubsystem extends SubsystemBase implements Component{
    
    private TalonFX shooterMotor1;
    private TalonFX shooterMotor2;
    private CANSparkMax transferMotor;
    private double acceleration;
    private double maxSpeed;
    private boolean isPriming;
    private double targetSpeed;

    public ShooterSubsystem(){
        shooterMotor1 = new TalonFX(Constants.Shooter.leftShooterMotorID);
        shooterMotor2 = new TalonFX(Constants.Shooter.rightShooterMotorID);
        transferMotor = new CANSparkMax(Constants.Shooter.transferMotorID, MotorType.kBrushless);
    }
    public void shoot(double speed){
        shooterMotor1.set(speed);
        shooterMotor2.set(speed);
    }
    public void transfer(double speed){
        transferMotor.set(speed);
    }
    
    public void stop(){
        shooterMotor1.set(0);
        shooterMotor2.set(0);
        transferMotor.set(0);
    }

    public double getShooterSpeed(){
        return (shooterMotor1.get()+shooterMotor2.get())/2;
    }
    @Override
    public void periodic() {

        //CurrentManager.updateCurrent(1, CurrentManager.Subsystem.Shooter);
        SmartDashboard.putNumber("Total Shooter Current Draw", shooterMotor1.getSupplyCurrent().getValue() + shooterMotor2.getSupplyCurrent().getValue()  + transferMotor.getOutputCurrent());
        SmartDashboard.putNumber("Shooter Speed", getShooterSpeed());
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
