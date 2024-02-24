package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class IntakeSubsystem extends SubsystemBase implements Component {

    private final CANSparkMax leftSwivelMotor;
    private final CANSparkMax rightSwivelMotor;

    private final TalonFX intakeMotor;

    private final SparkAbsoluteEncoder angleEncoder;

    private final ArmFeedforward swivelFeedforwardController;
    private final PIDController swivelPIDController;

    private double swivelSetpoint;
    
    public IntakeSubsystem(){
        leftSwivelMotor = new CANSparkMax(Constants.Intake.leftSwivelMotorID, MotorType.kBrushless);
        rightSwivelMotor = new CANSparkMax(Constants.Intake.rightSwivelMotorID, MotorType.kBrushless);

        leftSwivelMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rightSwivelMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        angleEncoder = leftSwivelMotor.getAbsoluteEncoder();

        intakeMotor = new TalonFX(Constants.Intake.intakeMotorID);

        swivelPIDController = new PIDController(Constants.Intake.kPSwivelController, Constants.Intake.kISwivelController, Constants.Intake.kDSwivelController);
        swivelFeedforwardController = new ArmFeedforward(Constants.Intake.kSSwivelFeedforward, Constants.Intake.kVSwivelFeedforward, Constants.Intake.kASwivelFeedforward);
    }
    
    public void setIntakeSpeed(double speed){
        intakeMotor.set(speed);
    }

    public void setSwivelPosition(double position){
        swivelSetpoint = position;
    }

    public void stopAll(){
        leftSwivelMotor.set(0);
        rightSwivelMotor.set(0);
        intakeMotor.set(0);
    }

    public double getSwivelPosition(){
        return angleEncoder.getPosition() % 1 * 360;
    }

    @Override
    public void periodic() {
        double currentAngle = getSwivelPosition();
        double pidOutput = swivelPIDController.calculate(currentAngle, swivelSetpoint);

        double targetAngleRadians = Math.toRadians(swivelSetpoint);

        double feedforwardOutput = swivelFeedforwardController.calculate(targetAngleRadians, 0, 0);

        double output = pidOutput + feedforwardOutput;

        leftSwivelMotor.setVoltage(output);
        leftSwivelMotor.setVoltage(output); // Follows motor1, inverted if necessary

        SmartDashboard.putNumber("Total Intake Current Draw", leftSwivelMotor.getOutputCurrent() + rightSwivelMotor.getOutputCurrent() + intakeMotor.getSupplyCurrent().getValue());
        SmartDashboard.putNumber("Intake Swivel Position", getSwivelPosition());
        //CurrentManager.updateCurrent(1, CurrentManager.Subsystem.Intake);
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
        return 2;
    }
    @Override
    public void updateBasedOnAllocatedCurrent(){
        //update motor controller based on allocated current
    }
}
