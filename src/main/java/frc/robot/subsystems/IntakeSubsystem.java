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
        swivelFeedforwardController = new ArmFeedforward(Constants.Intake.kSSwivelFeedforward, Constants.Intake.kVSwivelFeedforward, Constants.Intake.kASwivelFeedforward);

        intakePIDController = new PIDController(Constants.Intake.kPIntakeController, Constants.Intake.kIIntakeController, Constants.Intake.kDIntakeController);

        swivelSetpoint = getSwivelPosition();
        targetIntakeSpeed = 0;

        allocatedCurrent = 0;

        slider = Shuffleboard.getTab("intake")
        .add("Slider", 0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 12))
        .getEntry();
    }
    
    public void setIntakeSpeed(double speed){
        targetIntakeSpeed = speed / (2 * Math.PI * Constants.Intake.intakeRollerRadius); // convert from cm/s to rps
    }

    public void setSwivelPosition(double position){
        swivelSetpoint = position;
    }

    public void stopAll(){
        leftSwivelMotor.setVoltage(0);
        rightSwivelMotor.setVoltage(0);
        intakeMotor.set(0);
    }

    public double getSwivelPosition(){ // returns the angle of the swivel in degrees
        return angleEncoder.getPosition() % 360 - Constants.Intake.kSwivelAnglePrecisionOffset;
    }

    public double getIntakeSpeed(){
        return intakeMotor.getRotorVelocity().getValue();
    }

    @Override
    public void periodic() {
        double currentAngle = getSwivelPosition();
        rightSwivelMotor.setVoltage(slider.getDouble(0));
        leftSwivelMotor.setVoltage(slider.getDouble(0));

        if (Math.abs(currentAngle - swivelSetpoint) > 3){ //TODO: deadband of 3 degrees
            double maxVoltage = RobotController.getBatteryVoltage() * 0.95;
            double pidOutput = swivelPIDController.calculate(currentAngle, swivelSetpoint);

            double targetAngleRadians = Math.toRadians(swivelSetpoint);

            double feedforwardOutput = swivelFeedforwardController.calculate(targetAngleRadians, 0, 0);

            double combinedOutput = pidOutput + feedforwardOutput;

            combinedOutput = Math.min(combinedOutput, maxVoltage);
            combinedOutput = Math.max(combinedOutput, -maxVoltage);

            SmartDashboard.putString("Swivel Mode", "ACTIVE");

            SmartDashboard.putNumber("Swivel Output Voltage", combinedOutput);
            
            //leftSwivelMotor.setVoltage(output);
            //rightSwivelMotor.setVoltage(output);
        }else SmartDashboard.putString("Swivel Mode", "IDLE");

        double currentIntakeSpeed = getIntakeSpeed();

        if (Math.abs(targetIntakeSpeed - currentIntakeSpeed) >= 5){ // TODO: deadband of 5 rpm
            double pidOutput = intakePIDController.calculate(currentIntakeSpeed, targetIntakeSpeed);
            // the values should be in the range of -1 to 1 and it will be clamped in the motor's api
            intakeMotor.set(pidOutput);
            SmartDashboard.putString("Intake Mode", "ACTIVE");
        } else SmartDashboard.putString("Intake Mode", "IDLE");

        SmartDashboard.putNumber("Total Intake Current Draw", getCurrentDraw());
        SmartDashboard.putNumber("Intake Swivel Position", angleEncoder.getPosition());
        SmartDashboard.putNumber("Intake Speed", getIntakeSpeed());
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
