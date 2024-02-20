package frc.robot.subsystems;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.SignalLogger;

import java.util.Map;

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

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.networktables.GenericEntry;


public class ShooterSubsystem extends SubsystemBase implements Component{
    
    private TalonFX shooterMotor1;
    private TalonFX shooterMotor2;
    private CANSparkMax transferMotor;
    private double acceleration;
    private double maxSpeed;
    private boolean isPriming;
    private double targetSpeed;
    private GenericEntry sliderEntry;
    // private final MotionMagicVelocityVoltage motionMagicVelocityVoltage;
    private final SysIdRoutine routine;
    private final SimpleMotorFeedforward feedforward;

    public ShooterSubsystem(){
        shooterMotor1 = new TalonFX(Constants.Shooter.leftShooterMotorID);
        shooterMotor1.setNeutralMode(NeutralModeValue.Coast);

        shooterMotor2 = new TalonFX(Constants.Shooter.rightShooterMotorID, Constants.CANivoreID);
        

        // // in init function
         TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();

        // // set slot 0 gains
        // var slot0Configs = talonFXConfigs.Slot0;
        // slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
        // slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        // slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        // slot0Configs.kP = 0.11; // An error of 1 rps results in 0.11 V output
        // slot0Configs.kI = 0; // no output for integrated error
        // slot0Configs.kD = 0; // no output for error derivative

        // set Motion Magic Velocity settings
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicAcceleration = 400; // Target acceleration of 400 rps/s (0.25 seconds to max)
        motionMagicConfigs.MotionMagicJerk = 4000; // Target jerk of 4000 rps/s/s (0.1 seconds)

        shooterMotor1.getConfigurator().apply(talonFXConfigs);
        shooterMotor2.getConfigurator().apply(talonFXConfigs);

        motionMagicVelocityVoltage = new MotionMagicVelocityVoltage(0);

        transferMotor = new CANSparkMax(Constants.Shooter.transferMotorID, MotorType.kBrushless);

        // Set the logger to log to the first flashdrive plugged in
    SignalLogger.setPath("/media/sda1/");

    //     routine = new SysIdRoutine(
    //     new SysIdRoutine.Config(), 
    //     new SysIdRoutine.Mechanism(
    //         (Measure<Voltage> volts) -> {
    //         shooterMotor1.setVoltage(volts.in(Volts));
    //         System.out.println("Volts: " + volts.in(Volts));
    //     }, log -> {
    //         // Record a frame for the shooter motor.
    //         log.motor("shooter-wheel")
    //             .voltage(
    //                 m_appliedVoltage.mut_replace(
    //                     shooterMotor1.get() * RobotController.getBatteryVoltage(), Volts))
    //             .angularPosition(m_angle.mut_replace(m_shooterEncoder.getDistance(), Rotations))
    //             .angularVelocity(
    //                 m_velocity.mut_replace(m_shooterEncoder.getRate(), RotationsPerSecond));
    //       }, this));
    
        routine = new SysIdRoutine(
            new SysIdRoutine.Config(), 
            new SysIdRoutine.Mechanism(
                (Measure<Voltage> volts) -> {
                shooterMotor1.setVoltage(volts.in(Volts));
                System.out.println("Volts: " + volts.in(Volts));
            }, null, this));
    }

    public void shoot(double speed){
        // shooterMotor1.set(speed);
        // shooterMotor2.set(speed);
        
        // shooterMotor1.setControl(motionMagicVelocityVoltage.withVelocity(speed));
        // shooterMotor2.setControl(motionMagicVelocityVoltage.withVelocity(speed));
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
        shooterMotor1.setVoltage(sliderEntry.getDouble(0));


        //CurrentManager.updateCurrent(1, CurrentManager.Subsystem.Shooter);
        SmartDashboard.putNumber("Total Shooter Current Draw", shooterMotor1.getSupplyCurrent().getValue() + shooterMotor2.getSupplyCurrent().getValue()  + transferMotor.getOutputCurrent());
        SmartDashboard.putNumber("Shooter Left Velocity", Math.abs(shooterMotor1.getRotorVelocity().getValue()));
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction);
    }
    
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction);
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
