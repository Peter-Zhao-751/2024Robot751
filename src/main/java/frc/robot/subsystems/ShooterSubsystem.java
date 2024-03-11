package frc.robot.subsystems;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utility.TelemetryUpdater;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.estimator.KalmanFilter;


public class ShooterSubsystem extends SubsystemBase implements Component {
    
    private final TalonFX leftShooterMotor;
    private final TalonFX rightShooterMotor;

    private double targetSpeed;

    private final MotionMagicVelocityVoltage motionMagicVelocityVoltage;

    private final LinearSystem<N1, N1, N1> flyWheelPlant;

    private final KalmanFilter<N1, N1, N1> kalmanFilter;

    private final double allocatedCurrent;


    public ShooterSubsystem(){
        leftShooterMotor = new TalonFX(Constants.Shooter.leftShooterMotorID, Constants.CANivoreID);
        leftShooterMotor.setNeutralMode(NeutralModeValue.Coast);

        rightShooterMotor = new TalonFX(Constants.Shooter.rightShooterMotorID, Constants.CANivoreID);
        rightShooterMotor.setNeutralMode(NeutralModeValue.Coast);

        flyWheelPlant = LinearSystemId.identifyVelocitySystem(Constants.Shooter.kVFlyWheelFeedforward, Constants.Shooter.kAFlyWheelFeedforward);

        kalmanFilter = new KalmanFilter<>(Nat.N1(), Nat.N1(), flyWheelPlant, VecBuilder.fill(Constants.Shooter.kProcessNoise), VecBuilder.fill(Constants.Shooter.kMeasurementNoise), 0.020);

        // in init function
        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();

        // set slot 0 gains
        Slot0Configs slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = Constants.Shooter.kSFlyWheelFeedforward;
        slot0Configs.kV = Constants.Shooter.kVFlyWheelFeedforward;
        slot0Configs.kA = Constants.Shooter.kAFlyWheelFeedforward;
        slot0Configs.kP = Constants.Shooter.kPFlyWheelController;
        slot0Configs.kI = Constants.Shooter.kIFlyWheelController;
        slot0Configs.kD = Constants.Shooter.kDFlyWheelController;

        // set Motion Magic Velocity settings
        MotionMagicConfigs motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicAcceleration = Constants.Shooter.motionMagicAcceleration; // Target acceleration of 400 rps/s (0.25 seconds to max)
        motionMagicConfigs.MotionMagicJerk = Constants.Shooter.motionMagicJerk; // Target jerk of 4000 rps/s/s (0.1 seconds)

        leftShooterMotor.getConfigurator().apply(talonFXConfigs);
        rightShooterMotor.getConfigurator().apply(talonFXConfigs);

        targetSpeed = 0;

        motionMagicVelocityVoltage = new MotionMagicVelocityVoltage(0);

        allocatedCurrent = 0;
    }

    /**
     * <p> Sets the speed of the shooter motors in rotations per second using the motion magic control mode </p> 
     * <ul>
     * <li> Targets the given velocity and uses the feedforward gains to achieve the target speed </li> 
     * <li> Additionally, reduces the acceleration to target a certain jerk </li> 
     * </ul>
     *
     * @param speed Set the speed of the shooter motors in rotations per second
     */
    public void setSpeed(double speed){
        targetSpeed = speed;
        leftShooterMotor.setControl(motionMagicVelocityVoltage.withVelocity(-speed));
        rightShooterMotor.setControl(new Follower(leftShooterMotor.getDeviceID(), true));
    }
    
    /**
     * <p> Stops the shooter motors </p> 
     * <ul>
     * <li> Sets the target speed to 0 </li> 
     * <li> Sets the voltage of the shooter motors to 0 </li> 
     * </ul>
     *
     */
    public void stop(){
        setSpeed(0);
    }

    /**
     * <p> Returns the speed of the left shooter motor in rotations per second </p> 
     *
     * @return double, the speed of the left shooter motor in rotations per second
     */
    public double getShooterMotor1Speed(){
        return leftShooterMotor.getRotorVelocity().getValue();
    }

    /**
     * <p> Returns the speed of the right shooter motor in rotations per second </p> 
     *
     * @return double, the speed of the right shooter motor in rotations per second
     */
    public double getShooterMotor2Speed(){
        return rightShooterMotor.getRotorVelocity().getValue();
    }

    /**
     * <p> Returns the average speed of the shooter motors in rotations per second </p> 
     *
     * @return double
     */
    public double getShooterSpeed(){
        return (getShooterMotor1Speed() + getShooterMotor2Speed()) / 2;
    }

    public double getEstimatedSpinUpTime(double speed){
        return 0.2;
    }

    @Override
    public void periodic() {

        kalmanFilter.correct(VecBuilder.fill(targetSpeed), VecBuilder.fill(getShooterMotor1Speed()));

        //TelemetryUpdater.setTelemetryValue("Kalman Filter X-hat 0", kalmanFilter.getXhat(0));
        //TelemetryUpdater.setTelemetryValue("Shooter Current Draw", getCurrentDraw());
    }

    @Override
    public double getCurrentDraw(){
        return leftShooterMotor.getSupplyCurrent().getValue() + rightShooterMotor.getSupplyCurrent().getValue();
    }

    @Override
    public void allocateCurrent(double current){
        //set motor controller current
    }

    @Override
    public int getPriority(){
        return 1;
    }
}
