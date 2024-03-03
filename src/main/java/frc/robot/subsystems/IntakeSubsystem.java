package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

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

    private double allocatedCurrent;
    
    public IntakeSubsystem(){
        leftSwivelMotor = new CANSparkMax(Constants.Intake.leftSwivelMotorID, MotorType.kBrushless);
        rightSwivelMotor = new CANSparkMax(Constants.Intake.rightSwivelMotorID, MotorType.kBrushless);

        leftSwivelMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rightSwivelMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        angleEncoder = leftSwivelMotor.getAbsoluteEncoder(Type.kDutyCycle);
        angleEncoder.setZeroOffset(Constants.Intake.k);

        intakeMotor = new TalonFX(Constants.Intake.intakeMotorID);

        swivelPIDController = new PIDController(Constants.Intake.kPSwivelController, Constants.Intake.kISwivelController, Constants.Intake.kDSwivelController);
        swivelFeedforwardController = new ArmFeedforward(Constants.Intake.kSSwivelFeedforward, Constants.Intake.kVSwivelFeedforward, Constants.Intake.kASwivelFeedforward);

        intakePIDController = new PIDController(Constants.Intake.kPIntakeController, Constants.Intake.kIIntakeController, Constants.Intake.kDIntakeController);

        swivelSetpoint = getSwivelPosition();
        targetIntakeSpeed = 0;
        allocatedCurrent = 0;
    }
    
    public void setIntakeSpeed(double speed){
        targetIntakeSpeed = speed / (2 * Math.PI * Constants.Intake.intakeRollerRadius) * 60; // convert from cm/s to rpm
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
        return angleEncoder.getPosition() % 1 * 360;
    }

    public double getIntakeSpeed(){
        return intakeMotor.getRotorVelocity().getValue();
    }

    @Override
    public void periodic() {
        double currentAngle = getSwivelPosition();

        if (Math.abs(currentAngle - swivelSetpoint) > 3){ //TODO: deadband of 3 degrees
            double pidOutput = swivelPIDController.calculate(currentAngle, swivelSetpoint);

            double targetAngleRadians = Math.toRadians(swivelSetpoint);

            double feedforwardOutput = swivelFeedforwardController.calculate(targetAngleRadians, 0, 0);

            double output = pidOutput + feedforwardOutput;

            leftSwivelMotor.setVoltage(output);
            rightSwivelMotor.setVoltage(output);
        }

        if (Math.abs(targetIntakeSpeed - 0) >= 5){ // TODO: deadband of 5 rpm
            double pidOutput = intakePIDController.calculate(intakeMotor.getRotorVelocity().getValue(), targetIntakeSpeed);
            intakeMotor.setVoltage(pidOutput);
        }

        SmartDashboard.putNumber("Total Intake Current Draw", getCurrentDraw());
        SmartDashboard.putNumber("Intake Swivel Position", getSwivelPosition());
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
