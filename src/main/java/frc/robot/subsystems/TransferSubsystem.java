package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.utility.TelemetryUpdater;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TransferSubsystem extends SubsystemBase implements Component {
    private final CANSparkMax shooterTransfer;
    private final CANSparkMax intakeTransfer;
    private final DigitalInput beamBreak;

    private final PIDController shooterTransferPIDController;
    private final PIDController intakeTransferPIDController;

    private double targetIntakeSpeed;
    private double targetShooterSpeed;

    private double allocatedCurrent;

    public TransferSubsystem() {
        shooterTransfer = new CANSparkMax(Constants.Transfer.shooterTransferID, MotorType.kBrushless);
        intakeTransfer = new CANSparkMax(Constants.Transfer.intakeTransferID, MotorType.kBrushless);
        intakeTransfer.setInverted(true);
        shooterTransfer.setInverted(true);
        beamBreak = new DigitalInput(Constants.Transfer.beamBreakDIOPort);

        shooterTransferPIDController = new PIDController(Constants.Transfer.kPIntakeController, 0, 0);
        intakeTransferPIDController = new PIDController(Constants.Transfer.kPShooterController, 0, 0);
        
        targetIntakeSpeed = 0;
        targetShooterSpeed = 0;

        allocatedCurrent = 0;
    }

    /**
     * Set the speed of the intake transfer motor
     * @param speed in centimeters per second
     */
    public void setIntakeTransfer(double speed) { // cm/s
        targetIntakeSpeed = speed / (2 * Math.PI * Constants.Transfer.intakeTransferRadius);
    }

    /**
     * Set the speed of the shooter transfer motor
     * @param speed in centimeters per second
     */
    public void setShooterTransfer(double speed) {
        targetShooterSpeed = speed / (2 * Math.PI * Constants.Transfer.shooterTransferRadius);
    }

    /**
     * Set the speed of both transfer motors
     * @param speed in centimeters per second
     */
    public void setTransferSpeed(double speed) { // in centimeters per second
        setIntakeTransfer(speed);
        setShooterTransfer(speed);
    }

    /**
     * Stop both transfer motors
     */
    public void stop() {
        targetIntakeSpeed = 0;
        targetShooterSpeed = 0;

        // TODO: should not be needed
        // intakeTransfer.stopMotor(); 
        // shooterTransfer.stopMotor();
    }

    /**
     * Returns if the beam is broken
     * @return boolean, true if the beam is broken
     */
    public boolean beamBroken() { // True when beam is broken
        return !beamBreak.get();
    }

    public double getIntakeSpeed() { // rps
        return intakeTransfer.getEncoder().getVelocity() / 60;
    }

    public double getShooterSpeed() { // rps
        return shooterTransfer.getEncoder().getVelocity() / 60;
    }


    @Override
    public void periodic() {
        //TelemetryUpdater.setTelemetryValue("Beam Break", beamBroken());
        //TelemetryUpdater.setTelemetryValue("Transfer Current Draw", getCurrentDraw());

        TelemetryUpdater.setTelemetryValue("Intake Speed", getIntakeSpeed());
        TelemetryUpdater.setTelemetryValue("Shooter Speed", getShooterSpeed());
        TelemetryUpdater.setTelemetryValue("Intake Target Speed", targetIntakeSpeed);
        TelemetryUpdater.setTelemetryValue("Shooter Target Speed", targetShooterSpeed);

        double currentIntakeSpeed = intakeTransfer.getEncoder().getVelocity() / 60;
        double intakeOutput = intakeTransferPIDController.calculate(currentIntakeSpeed, targetIntakeSpeed);
        intakeTransfer.set(intakeOutput);

        double currentShooterSpeed = shooterTransfer.getEncoder().getVelocity() / 60;
        double shooterOutput = shooterTransferPIDController.calculate(currentShooterSpeed, targetShooterSpeed);
        shooterTransfer.set(shooterOutput);
    }

    @Override
    public double getCurrentDraw() {
        return shooterTransfer.getOutputCurrent() + intakeTransfer.getOutputCurrent();
    }

    @Override
    public void allocateCurrent(double current) {
    }

    @Override
    public int getPriority() {
        return 5; // CHANGE
    }
}
