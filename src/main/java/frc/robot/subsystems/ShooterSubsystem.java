package frc.robot.subsystems;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
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
import edu.wpi.first.math.estimator.KalmanFilter;

// // Logging stuff for Characterization
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
// import com.ctre.phoenix6.SignalLogger;
// import edu.wpi.first.units.Measure;
// import edu.wpi.first.units.Voltage;
// import static edu.wpi.first.units.Units.Volts;

// https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-flywheel-walkthrough.html#modeling-with-system-identification
// https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-observers.html

public class ShooterSubsystem extends SubsystemBase implements Component {
    
    private final TalonFX shooterMotor1;
    private final TalonFX shooterMotor2;

    private double targetSpeed;

    // private final double acceleration;
    // private final double maxSpeed;
    // private final boolean isPriming;
    // private final double targetSpeed;
    private final MotionMagicVelocityVoltage motionMagicVelocityVoltage;

    private final LinearSystem<N1, N1, N1> flyWheelPlant;

    private final KalmanFilter<N1, N1, N1> kalmanFilter;



    // private final SysIdRoutine routine;

    public ShooterSubsystem(){
        shooterMotor1 = new TalonFX(Constants.Shooter.leftShooterMotorID);
        shooterMotor1.setNeutralMode(NeutralModeValue.Coast);

        shooterMotor2 = new TalonFX(Constants.Shooter.rightShooterMotorID, Constants.CANivoreID);
        shooterMotor2.setNeutralMode(NeutralModeValue.Coast);

        flyWheelPlant = LinearSystemId.identifyVelocitySystem(Constants.Shooter.kVFlyWheelFeedforward, Constants.Shooter.kAFlyWheelFeedforward);

        kalmanFilter = new KalmanFilter<>(Nat.N1(), Nat.N1(), flyWheelPlant, VecBuilder.fill(Constants.Shooter.kProcessNoise), VecBuilder.fill(Constants.Shooter.kMeasurementNoise), 0.020);

        // in init function
        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();

        // set slot 0 gains
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = Constants.Shooter.kSFlyWheelFeedforward;
        slot0Configs.kV = Constants.Shooter.kVFlyWheelFeedforward;
        slot0Configs.kA = Constants.Shooter.kAFlyWheelFeedforward;
        slot0Configs.kP = Constants.Shooter.kPFlyWheelController;
        slot0Configs.kI = Constants.Shooter.kIFlyWheelController;
        slot0Configs.kD = Constants.Shooter.kDFlyWheelController;

        // set Motion Magic Velocity settings
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicAcceleration = 40; // Target acceleration of 400 rps/s (0.25 seconds to max)
        motionMagicConfigs.MotionMagicJerk = 400; // Target jerk of 4000 rps/s/s (0.1 seconds)

        shooterMotor1.getConfigurator().apply(talonFXConfigs);
        shooterMotor2.getConfigurator().apply(talonFXConfigs);

        targetSpeed = 0;

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

    public void setSpeed(double speed){
        targetSpeed = speed;
        shooterMotor1.setControl(motionMagicVelocityVoltage.withVelocity(-speed));
        shooterMotor2.setControl(new Follower(shooterMotor1.getDeviceID(), true));
    }
    
    public void stop(){
        targetSpeed = 0;
        shooterMotor1.set(0);
        shooterMotor2.set(0);
    }

    public double getShooterMotor1Speed(){
        return shooterMotor1.getRotorVelocity().getValue();
    }
    public double getShooterMotor2Speed(){
        return shooterMotor2.getRotorVelocity().getValue();
    }

    public double getShooterSpeed(){
        return (getShooterMotor1Speed() + getShooterMotor2Speed()) / 2;
    }

    public double getEstimatedSpinUpTime(double speed){
        return 0.2;
    }

    @Override
    public void periodic() {


        //CurrentManager.updateCurrent(1, CurrentManager.Subsystem.Shooter);

        ShuffleboardTab tab = Shuffleboard.getTab("Shooter");

        tab.add("Shooter Velocity", Math.abs(shooterMotor1.getRotorVelocity().getValue() + shooterMotor2.getRotorVelocity().getValue()) / 2);
        tab.add("Shooter Voltage", Math.abs(shooterMotor1.getMotorVoltage().getValue() + shooterMotor2.getMotorVoltage().getValue()) / 2);
        tab.add("Shooter Current", Math.abs(shooterMotor1.getSupplyCurrent().getValue() + shooterMotor2.getSupplyCurrent().getValue()) / 2);

        tab.add("Shooter Left Velocity", Math.abs(shooterMotor1.getRotorVelocity().getValue()));
        tab.add("Shooter Left Voltage", Math.abs(shooterMotor1.getMotorVoltage().getValue()));
        tab.add("Shooter Left Current", Math.abs(shooterMotor1.getSupplyCurrent().getValue()));

        tab.add("Shooter Right Velocity", Math.abs(shooterMotor2.getRotorVelocity().getValue()));
        tab.add("Shooter Right Voltage", Math.abs(shooterMotor2.getMotorVoltage().getValue()));
        tab.add("Shooter Right Current", Math.abs(shooterMotor2.getSupplyCurrent().getValue()));

        kalmanFilter.correct(VecBuilder.fill(targetSpeed), VecBuilder.fill(getShooterMotor1Speed()));

        tab.add("Kalman Filter X-hat 0", kalmanFilter.getXhat(0));
        try{
            tab.add("Kalman Filter X-hat 1", kalmanFilter.getXhat(1));
        } catch (Exception e){
            tab.add("Kalman Filter X-hat 1", "dumb");
        }
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
        return 1;
    }

    @Override
    public void updateBasedOnAllocatedCurrent(){
        //update motor controller based on allocated current
    }
}
