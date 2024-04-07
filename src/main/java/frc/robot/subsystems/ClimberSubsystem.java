package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.utility.TelemetryUpdater;

public class ClimberSubsystem extends SubsystemBase{
    private static ClimberSubsystem instance;

    private final CANSparkMax leftClimberMotor;
    private final CANSparkMax rightClimberMotor;

    private final RelativeEncoder leftClimberEncoder;
    private final RelativeEncoder rightClimberEncoder;

    private double leftDesiredVoltage;
    private double rightDesiredVoltage;

    public static ClimberSubsystem getInstance() {
        if(instance == null) instance = new ClimberSubsystem();
        return instance;
    }

    private ClimberSubsystem() {
        leftClimberMotor = new CANSparkMax(Constants.Climber.leftClimberMotorID, MotorType.kBrushless);
        rightClimberMotor = new CANSparkMax(Constants.Climber.rightClimberMotorID, MotorType.kBrushless);

        leftClimberEncoder = leftClimberMotor.getEncoder();
        rightClimberEncoder = rightClimberMotor.getEncoder();

        //leftClimberMotor.setInverted(true);
        leftClimberMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        leftClimberMotor.setSmartCurrentLimit(80);
		rightClimberMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rightClimberMotor.setSmartCurrentLimit(80);

        leftDesiredVoltage = 0;
        rightDesiredVoltage = 0;
    }

    private double getLeftPosition(){
        return leftClimberEncoder.getPosition() / Constants.Climber.kGearRatio * 2 * Constants.Climber.kSpoolRadius * Math.PI;
    }

    private double getRightPosition(){
        return rightClimberEncoder.getPosition() / Constants.Climber.kGearRatio * 2 * Constants.Climber.kSpoolRadius * Math.PI;
    }

    public void setRightVoltage(double volts) {
        rightClimberMotor.setVoltage(volts);
    }

    public void setLeftVoltage(double volts) {
        leftClimberMotor.setVoltage(volts);
    }

    /**
     * Stop both climber motors
     */
    public void stop(){
		// leftClimberMotor.stopMotor();
		// rightClimberMotor.stopMotor();
		leftDesiredVoltage = 0;
		rightDesiredVoltage = 0;
    }

    @Override
    public void periodic() {
        TelemetryUpdater.setTelemetryValue("Climber/Left Climber Current", leftClimberMotor.getOutputCurrent());
		TelemetryUpdater.setTelemetryValue("Climber/Right Climber Current", rightClimberMotor.getOutputCurrent());

		TelemetryUpdater.setTelemetryValue("Climber/Left Climber Position", getLeftPosition());
		TelemetryUpdater.setTelemetryValue("Climber/Right Climber Position", getRightPosition());

        /*TelemetryUpdater.setTelemetryValue("Climber Current Draw", getCurrentDraw());
        TelemetryUpdater.setTelemetryValue("Left Climber Position", getLeftPosition());
        TelemetryUpdater.setTelemetryValue("Right Climber Position", getRightPosition());*/
    }
}
