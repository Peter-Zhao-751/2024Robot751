package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utility.TelemetryUpdater;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import edu.wpi.first.wpilibj.RobotController;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import static edu.wpi.first.units.Units.Volts;

public class IntakeSubsystem extends SubsystemBase implements Component {

    private final CANSparkMax leftSwivelMotor;
    private final CANSparkMax rightSwivelMotor;

    private final TalonFX intakeMotor;
    private final DutyCycleOut dutyCycleOut;
    private final VelocityVoltage velocityVoltage;

    private final SparkAbsoluteEncoder angleEncoder;
    TrapezoidProfile trapezoidProfile;


    private final ArmFeedforward swivelFeedforwardController;
    private final PIDController swivelPIDController;
    private final PIDController intakePIDController;
 
    private double swivelSetpoint;
    private double targetIntakeSpeed;
    private GenericEntry slider;

    private double allocatedCurrent;
    private double startTime;
    private double startAngle;

//    private SysIdRoutine routine;
    
    public IntakeSubsystem(){
        leftSwivelMotor = new CANSparkMax(Constants.Intake.leftSwivelMotorID, MotorType.kBrushless);
        rightSwivelMotor = new CANSparkMax(Constants.Intake.rightSwivelMotorID, MotorType.kBrushless);

        leftSwivelMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        rightSwivelMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);

        leftSwivelMotor.setInverted(true);

        angleEncoder = rightSwivelMotor.getAbsoluteEncoder(Type.kDutyCycle);
        angleEncoder.setInverted(true);
        angleEncoder.setPositionConversionFactor(360);
        angleEncoder.setZeroOffset(Constants.Intake.kSwivelEncoderZeroOffset);

        intakeMotor = new TalonFX(Constants.Intake.intakeMotorID);
        dutyCycleOut = new DutyCycleOut(0);
        TalonFXConfiguration intakeMotorConfig = new TalonFXConfiguration();
        Slot0Configs slot0 = intakeMotorConfig.Slot0;
        slot0.kS = Constants.Intake.kSIntakeController;
        slot0.kV = Constants.Intake.kVIntakeController;
        slot0.kP = Constants.Intake.kPIntakeController;
        slot0.kI = Constants.Intake.kIIntakeController;
        slot0.kD = Constants.Intake.kDIntakeController;
        intakeMotor.getConfigurator().apply(slot0);


        velocityVoltage = new VelocityVoltage(0);

        swivelPIDController = new PIDController(Constants.Intake.kPSwivelController, Constants.Intake.kISwivelController, Constants.Intake.kDSwivelController);
        swivelPIDController.enableContinuousInput(0, 360);
        swivelFeedforwardController = new ArmFeedforward(Constants.Intake.kSSwivelFeedforward, Constants.Intake.kGSwivelFeedforward, Constants.Intake.kVSwivelFeedforward);

        intakePIDController = new PIDController(Constants.Intake.kPIntakeController, Constants.Intake.kIIntakeController, Constants.Intake.kDIntakeController);

        trapezoidProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(200, 400));

        swivelSetpoint = getSwivelPosition();
        targetIntakeSpeed = 0;

        allocatedCurrent = 0;

        startTime = System.currentTimeMillis();
        startAngle = getSwivelPosition();
        setSwivelPosition(Constants.Intake.IntakePositions.RETRACTED);

//        slider = Shuffleboard.getTab("intake")
//        .add("Slider", 0)
//        .withWidget(BuiltInWidgets.kNumberSlider)
//        .withProperties(Map.of("min", 0, "max", 12))
//        .getEntry();

//         routine = new SysIdRoutine(
//             new SysIdRoutine.Config(
//                 null,
//                 null,
//                 null,
//                 (state) -> SignalLogger.writeString("state", state.toString())
//             ),
//             new SysIdRoutine.Mechanism(
//                 (Measure<Voltage> volts) -> {
//                 intakeMotor.setVoltage(volts.in(Volts));
//                 System.out.println("Volts: " + volts.in(Volts));
//             }, null, this)
//         );
    }

    // public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    //     return routine.quasistatic(direction);
    // }

    // public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    //     return routine.dynamic(direction);
    // }
    
    /**
     * Sets the speed of the intake motor
     * @param speed the speed of the intake motor in cm/s
     */
    public void setIntakeSpeed(double speed){
        //targetIntakeSpeed = speed / (2 * Math.PI * Constants.Intake.intakeRollerRadius);
        intakeMotor.set(0.25);
    }

    /**
     * Sets the position of the swivel
     * @param position the position of the swivel in degrees
     */
    public void setSwivelPosition(Constants.Intake.IntakePositions position){
        swivelSetpoint = position.getAngle();
        startAngle = getSwivelPosition();
        startTime = System.currentTimeMillis();
    }

    /**
     * Stops the intake and swivel motors
     */
    public void stopAll(){
        leftSwivelMotor.stopMotor();
        rightSwivelMotor.stopMotor();
        targetIntakeSpeed = 0;
        intakeMotor.stopMotor();
    }

    /**
     * Returns the position of the swivel in degrees
     * @return double, the position of the swivel in degrees
     */
    public double getSwivelPosition() { 
        double currentAngle = angleEncoder.getPosition() % 360;
        if (currentAngle >= 180) return currentAngle - 360;
        return currentAngle;
    }

    /**
     * Returns the speed of the intake motor in rotations per second
     * @return double, the speed of the intake motor in rotations per second
     */
    public double getIntakeSpeed(){
        return intakeMotor.getRotorVelocity().getValue();
    }

    @Override
    public void periodic() {
        double deltaTime = (System.currentTimeMillis() - startTime) / 1000;
        double currentAngle = getSwivelPosition();

        // debug shit
        //rightSwivelMotor.setVoltage(slider.getDouble(0));
        //leftSwivelMotor.setVoltage(slider.getDouble(0));

        TrapezoidProfile.State setPoint = trapezoidProfile.calculate(deltaTime, new TrapezoidProfile.State(startAngle, 0), new TrapezoidProfile.State(swivelSetpoint, 0));

        double maxVoltage = RobotController.getBatteryVoltage() * 0.95; // TODO: maybe replace with the PDH voltage?

        double feedforwardOutput = swivelFeedforwardController.calculate(Math.toRadians(setPoint.position), Math.toRadians(setPoint.velocity), 0);
        double swivelPidOutput = swivelPIDController.calculate(currentAngle, setPoint.position);
        
        double combinedOutput = feedforwardOutput + swivelPidOutput;

        // combinedOutput = Math.min(combinedOutput, 6);
        // combinedOutput = Math.max(combinedOutput, -6);

        // TelemetryUpdater.setTelemetryValue("Swivel Output Voltage", combinedOutput);
        // TelemetryUpdater.setTelemetryValue("PID output voltage", swivelPidOutput);
        // TelemetryUpdater.setTelemetryValue("setpoint trapezoidal pos", setPoint.position);
        // TelemetryUpdater.setTelemetryValue("setpoint trapezoidal vel", setPoint.velocity);
        // TelemetryUpdater.setTelemetryValue("Trapezoidal ETA", trapezoidProfile.totalTime());
        
        leftSwivelMotor.setVoltage(combinedOutput);
        rightSwivelMotor.setVoltage(combinedOutput);

        //double intakePidOutput = intakePIDController.calculate(getIntakeSpeed(), targetIntakeSpeed);
        // the values should be in the range of -1 to 1, and it will be clamped in the motor's api
        //intakeMotor.set(targetIntakeSpeed);

        // TelemetryUpdater.setTelemetryValue("Total Intake Current Draw", getCurrentDraw());
        TelemetryUpdater.setTelemetryValue("Intake Swivel Position", currentAngle);
        //TelemetryUpdater.setTelemetryValue("setpoint swivel", swivelSetpoint);
        // TelemetryUpdater.setTelemetryValue("Intake Speed", getIntakeSpeed());
    }

    @Override
    public double getCurrentDraw(){
        return leftSwivelMotor.getOutputCurrent() + rightSwivelMotor.getOutputCurrent() + intakeMotor.getSupplyCurrent().getValue();
    }

    @Override
    public void allocateCurrent(double current){
        //set motor controller current
    }

    @Override
    public int getPriority(){
        return 2;
    }
}
