package frc.robot.subsystems;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import frc.robot.Constants;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
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
    private final MotionMagicVelocityVoltage motionMagicVelocityVoltage;

    public ShooterSubsystem(){
        shooterMotor1 = new TalonFX(Constants.Shooter.leftShooterMotorID);
        shooterMotor1.setNeutralMode(NeutralModeValue.Coast);

        shooterMotor2 = new TalonFX(Constants.Shooter.rightShooterMotorID);
        

        // in init function
        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();

        // set slot 0 gains
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.kP = 0.11; // An error of 1 rps results in 0.11 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0; // no output for error derivative

        // set Motion Magic Velocity settings
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicAcceleration = 400; // Target acceleration of 400 rps/s (0.25 seconds to max)
        motionMagicConfigs.MotionMagicJerk = 4000; // Target jerk of 4000 rps/s/s (0.1 seconds)

        shooterMotor1.getConfigurator().apply(talonFXConfigs);
        shooterMotor2.getConfigurator().apply(talonFXConfigs);

        motionMagicVelocityVoltage = new MotionMagicVelocityVoltage(0);

        transferMotor = new CANSparkMax(Constants.Shooter.transferMotorID, MotorType.kBrushless);
    }
    public void shoot(double speed){
        // shooterMotor1.set(speed);
        // shooterMotor2.set(speed);
        
        shooterMotor1.setControl(motionMagicVelocityVoltage.withVelocity(speed));
        shooterMotor2.setControl(motionMagicVelocityVoltage.withVelocity(speed));
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
