package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import edu.wpi.first.wpilibj.RobotController;

import java.util.Map;

public class IntakeSubsystem extends SubsystemBase implements Component {

    private final CANSparkMax leftSwivelMotor;
    private final CANSparkMax rightSwivelMotor;

    private final TalonFX intakeMotor;

    private final SparkAbsoluteEncoder angleEncoder;

    private final ArmFeedforward swivelFeedforwardController;
    private final PIDController swivelPIDController;
    private final PIDController intakePIDController;
 
    private double swivelSetpoint;
    private double targetIntakeSpeed;
    private GenericEntry slider;

    private double allocatedCurrent;
    
    public IntakeSubsystem(){
        leftSwivelMotor = new CANSparkMax(Constants.Intake.leftSwivelMotorID, MotorType.kBrushless);
        rightSwivelMotor = new CANSparkMax(Constants.Intake.rightSwivelMotorID, MotorType.kBrushless);

        leftSwivelMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        rightSwivelMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        leftSwivelMotor.setInverted(true);

        angleEncoder = rightSwivelMotor.getAbsoluteEncoder(Type.kDutyCycle);
        angleEncoder.setInverted(true);
        angleEncoder.setPositionConversionFactor(360);

        intakeMotor = new TalonFX(Constants.Intake.intakeMotorID);

        swivelPIDController = new PIDController(Constants.Intake.kPSwivelController, Constants.Intake.kISwivelController, Constants.Intake.kDSwivelController);
        swivelPIDController.enableContinuousInput(0, 360);
        swivelFeedforwardController = new ArmFeedforward(Constants.Intake.kSSwivelFeedforward, Constants.Intake.kVSwivelFeedforward, Constants.Intake.kASwivelFeedforward);

        intakePIDController = new PIDController(Constants.Intake.kPIntakeController, Constants.Intake.kIIntakeController, Constants.Intake.kDIntakeController);

        swivelSetpoint = 50;
        targetIntakeSpeed = 0;

        allocatedCurrent = 0;

        slider = Shuffleboard.getTab("intake")
        .add("Slider", 0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 12))
        .getEntry();
    }
    
    /**
     * Sets the speed of the intake motor
     * @param speed the speed of the intake motor in cm/s
     * @return void
     */
    public void setIntakeSpeed(double speed){
        targetIntakeSpeed = speed / (2 * Math.PI * Constants.Intake.intakeRollerRadius);
    }

    /**
     * Sets the position of the swivel
     * @param position the position of the swivel in degrees
     * @return void
     */
    public void setSwivelPosition(double position){
        swivelSetpoint = position;
    }

    /**
     * Stops the intake and swivel motors
     * @return void
     */
    public void stopAll(){
        leftSwivelMotor.setVoltage(0);
        rightSwivelMotor.setVoltage(0);
        intakeMotor.set(0);
    }

    /**
     * Returns the position of the swivel in degrees
     * @return double, the position of the swivel in degrees
     */
    public double getSwivelPosition() { 
        return angleEncoder.getPosition() % 360 - Constants.Intake.kSwivelAnglePrecisionOffset;
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
        double currentAngle = getSwivelPosition();

        // debug shit
        //rightSwivelMotor.setVoltage(slider.getDouble(0));
        //leftSwivelMotor.setVoltage(slider.getDouble(0));

        double maxVoltage = RobotController.getBatteryVoltage() * 0.95;
        double swivelPidOutput = swivelPIDController.calculate(currentAngle, swivelSetpoint);
        
        double targetAngleRadians = Math.toRadians(swivelSetpoint);

        double feedforwardOutput = swivelFeedforwardController.calculate(targetAngleRadians, 0, 0);

        double combinedOutput = swivelPidOutput + feedforwardOutput;

        combinedOutput = Math.min(combinedOutput, 3);
        combinedOutput = Math.max(combinedOutput, -3);

        SmartDashboard.putNumber("Swivel Output Voltage", combinedOutput);
            
        leftSwivelMotor.setVoltage(combinedOutput);
        rightSwivelMotor.setVoltage(combinedOutput);

        double intakePidOutput = intakePIDController.calculate(getIntakeSpeed(), targetIntakeSpeed);
        // the values should be in the range of -1 to 1 and it will be clamped in the motor's api
        intakeMotor.set(intakePidOutput);

        // SmartDashboard.putNumber("Total Intake Current Draw", getCurrentDraw());
        SmartDashboard.putNumber("Intake Swivel Position", angleEncoder.getPosition());
        // SmartDashboard.putNumber("Intake Speed", getIntakeSpeed());
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
