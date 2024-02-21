package frc.robot.subsystems;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

// // Logging stuff for Characterization
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
// import com.ctre.phoenix6.SignalLogger;
// import edu.wpi.first.units.Measure;
// import edu.wpi.first.units.Voltage;
// import static edu.wpi.first.units.Units.Volts;

public class ShooterSubsystem extends SubsystemBase implements Component {
    
    private final TalonFX shooterMotor1;
    private final TalonFX shooterMotor2;
    // private final double acceleration;
    // private final double maxSpeed;
    // private final boolean isPriming;
    // private final double targetSpeed;
    private final MotionMagicVelocityVoltage motionMagicVelocityVoltage;
    // private final SysIdRoutine routine;

    public ShooterSubsystem(){
        shooterMotor1 = new TalonFX(Constants.Shooter.leftShooterMotorID, Constants.CANivoreID);
        shooterMotor1.setNeutralMode(NeutralModeValue.Coast);

        shooterMotor2 = new TalonFX(Constants.Shooter.rightShooterMotorID, Constants.CANivoreID);
        shooterMotor2.setNeutralMode(NeutralModeValue.Coast);
        

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
        motionMagicConfigs.MotionMagicAcceleration = 40; // Target acceleration of 400 rps/s (0.25 seconds to max)
        motionMagicConfigs.MotionMagicJerk = 400; // Target jerk of 4000 rps/s/s (0.1 seconds)

        shooterMotor1.getConfigurator().apply(talonFXConfigs);
        shooterMotor2.getConfigurator().apply(talonFXConfigs);

        motionMagicVelocityVoltage = new MotionMagicVelocityVoltage(0);
    
        // routine = new SysIdRoutine(
        //     new SysIdRoutine.Config(
        //         null,
        //         null,
        //         null,
        //         (state) -> SignalLogger.writeString("state", state.toString())
        //     ), 
        //     new SysIdRoutine.Mechanism(
        //         (Measure<Voltage> volts) -> {
        //         shooterMotor1.setVoltage(volts.in(Volts));
        //         System.out.println("Volts: " + volts.in(Volts));
        //     }, null, this)
        // );
    }

    // public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    //     return routine.quasistatic(direction);
    // }
    
    // public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    //     return routine.dynamic(direction);
    // }

    public void shoot(double speed){
        shooterMotor1.setControl(motionMagicVelocityVoltage.withVelocity(-speed));
        shooterMotor2.setControl(new Follower(shooterMotor1.getDeviceID(), true));
    }
    
    public void stop(){
        shooterMotor1.set(0);
        shooterMotor2.set(0);
    }

    public double getShooterSpeed(){
        return (shooterMotor1.get()+shooterMotor2.get())/2;
    }

    @Override
    public void periodic() {

        //CurrentManager.updateCurrent(1, CurrentManager.Subsystem.Shooter);

        // ShuffleboardTab tab = Shuffleboard.getTab("Shooter");

        // tab.add("Shooter Velocity", Math.abs(shooterMotor1.getRotorVelocity().getValue() + shooterMotor2.getRotorVelocity().getValue()) / 2);
        // tab.add("Shooter Voltage", Math.abs(shooterMotor1.getMotorVoltage().getValue() + shooterMotor2.getMotorVoltage().getValue()) / 2);
        // tab.add("Shooter Current", Math.abs(shooterMotor1.getSupplyCurrent().getValue() + shooterMotor2.getSupplyCurrent().getValue()) / 2);

        // tab.add("Shooter Left Velocity", Math.abs(shooterMotor1.getRotorVelocity().getValue()));
        // tab.add("Shooter Left Voltage", Math.abs(shooterMotor1.getMotorVoltage().getValue()));
        // tab.add("Shooter Left Current", Math.abs(shooterMotor1.getSupplyCurrent().getValue()));

        // tab.add("Shooter Right Velocity", Math.abs(shooterMotor2.getRotorVelocity().getValue()));
        // tab.add("Shooter Right Voltage", Math.abs(shooterMotor2.getMotorVoltage().getValue()));
        // tab.add("Shooter Right Current", Math.abs(shooterMotor2.getSupplyCurrent().getValue()));


        // tab.add("Transfer Velocity", Math.abs(transferMotor.getEncoder().getVelocity()));
        // tab.add("Transfer Voltage", Math.abs(transferMotor.getAppliedOutput()));
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
