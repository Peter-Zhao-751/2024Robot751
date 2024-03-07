package frc.robot.subsystems;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utility.TelemetryUpdater;

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
    
    private final TalonFX leftShooterMotor;
    private final TalonFX rightShooterMotor;

    private double targetSpeed;

    // private final double acceleration;
    // private final double maxSpeed;
    // private final boolean isPriming;
    // private final double targetSpeed;
    private final MotionMagicVelocityVoltage motionMagicVelocityVoltage;

    private final LinearSystem<N1, N1, N1> flyWheelPlant;

    private final KalmanFilter<N1, N1, N1> kalmanFilter;

    private final double allocatedCurrent;

    // private final SysIdRoutine routine;

    public ShooterSubsystem(){
        leftShooterMotor = new TalonFX(Constants.Shooter.leftShooterMotorID);
        leftShooterMotor.setNeutralMode(NeutralModeValue.Coast);

        rightShooterMotor = new TalonFX(Constants.Shooter.rightShooterMotorID, Constants.CANivoreID);
        rightShooterMotor.setNeutralMode(NeutralModeValue.Coast);

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

        leftShooterMotor.getConfigurator().apply(talonFXConfigs);
        rightShooterMotor.getConfigurator().apply(talonFXConfigs);

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

        allocatedCurrent = 0;
    }

    // public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    //     return routine.quasistatic(direction);
    // }
    
    // public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    //     return routine.dynamic(direction);
    // }

    /**
     * <p> Sets the speed of the shooter motors in rotations per second using the motion magic control mode </p> 
     * <ul>
     * <li> Targets the given velocity and uses the feedforward gains to achieve the target speed </li> 
     * <li> Additionally, reduces the acceleration to target a certain jerk </li> 
     * </ul>
     *
     * @param speed                Set the speed of the shooter motors in rotations per second
     * @return void
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
     * @return void
     */
    public void stop(){
        targetSpeed = 0;
        leftShooterMotor.set(0);
        rightShooterMotor.set(0);
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


        //CurrentManager.updateCurrent(1, CurrentManager.Subsystem.Shooter);

        //ShuffleboardTab tab = Shuffleboard.getTab("Robot Shooter132");

        // tab.add("Shooter Velocity", Math.abs(shooterMotor1.getRotorVelocity().getValue() + shooterMotor2.getRotorVelocity().getValue()) / 2);
        // tab.add("Shooter Voltage", Math.abs(shooterMotor1.getMotorVoltage().getValue() + shooterMotor2.getMotorVoltage().getValue()) / 2);
        // tab.add("Shooter Current", Math.abs(shooterMotor1.getSupplyCurrent().getValue() + shooterMotor2.getSupplyCurrent().getValue()) / 2);

        // tab.add("Shooter Left Velocity", Math.abs(shooterMotor1.getRotorVelocity().getValue()));
        // tab.add("Shooter Left Voltage", Math.abs(shooterMotor1.getMotorVoltage().getValue()));
        // tab.add("Shooter Left Current", Math.abs(shooterMotor1.getSupplyCurrent().getValue()));

        // tab.add("Shooter Right Velocity", Math.abs(shooterMotor2.getRotorVelocity().getValue()));
        // tab.add("Shooter Right Voltage", Math.abs(shooterMotor2.getMotorVoltage().getValue()));
        // tab.add("Shooter Right Current", Math.abs(shooterMotor2.getSupplyCurrent().getValue()));

        kalmanFilter.correct(VecBuilder.fill(targetSpeed), VecBuilder.fill(getShooterMotor1Speed()));

        TelemetryUpdater.setTelemetryValue("Kalman Filter X-hat 0", kalmanFilter.getXhat(0));
        TelemetryUpdater.setTelemetryValue("Shooter Current Draw", getCurrentDraw());
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
