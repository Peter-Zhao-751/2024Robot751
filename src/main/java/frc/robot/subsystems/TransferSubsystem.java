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

    private final double shooterTransferRadius = Constants.Transfer.shooterTransferRadius;
    private final double intakeTransferRadius = Constants.Transfer.intakeTransferRadius;

    private final PIDController shooterTransferPIDController;
    private final PIDController intakeTransferPIDController;

    private double allocatedCurrent;

    public TransferSubsystem() {
        shooterTransfer = new CANSparkMax(Constants.Transfer.shooterTransferID, MotorType.kBrushless);
        intakeTransfer = new CANSparkMax(Constants.Transfer.intakeTransferID, MotorType.kBrushless);
        beamBreak = new DigitalInput(Constants.Transfer.beamBreakDIOPort);

        shooterTransferPIDController = new PIDController(Constants.Transfer.kPIntakeController, 0, 0);
        intakeTransferPIDController = new PIDController(Constants.Transfer.kPShooterController, 0, 0);

        allocatedCurrent = 0;
    }

    /**
     * Set the speed of the intake transfer motor
     * @param speed in rotations per minute
     */
    public void setIntakeTransfer(double speed) { // in rpm
        // double currentSpeed = intakeTransfer.getEncoder().getVelocity() / 60;
        // double output = intakeTransferPIDController.calculate(currentSpeed, speed);
        System.out.println("jbadsadsad");
        intakeTransfer.set(0.5);
    }

    /**
     * Set the speed of the shooter transfer motor
     * @param speed in rotations per minute
     */
    public void setShooterTransfer(double speed) { // in rpm
        // double currentSpeed = shooterTransfer.getEncoder().getVelocity() / 60;
        // double output = shooterTransferPIDController.calculate(currentSpeed, speed);
        shooterTransfer.set(0.5);
    }

    /**
     * Set the speed of both transfer motors
     * @param speed in centimeters per second
     */
    public void transfer(double speed) { // in centimeters per second
        setIntakeTransfer(speed / (2 * Math.PI * intakeTransferRadius));
        setShooterTransfer(speed / (2 * Math.PI * shooterTransferRadius));
    }

    /**
     * Stop both transfer motors
     */
    public void stop() {
        intakeTransfer.set(0);
        shooterTransfer.set(0);
    }

    /**
     * Returns if the beam is broken
     * @return boolean, true if the beam is broken
     */
    public boolean beamBroken() { // True when beam is broken
        return !beamBreak.get();
    }

    @Override
    public void periodic() {
        TelemetryUpdater.setTelemetryValue("Beam Break", beamBroken());
        TelemetryUpdater.setTelemetryValue("Transfer Current Draw", getCurrentDraw());
        TelemetryUpdater.setTelemetryValue("Intake Transfer Speed", intakeTransfer.getEncoder().getVelocity());
        TelemetryUpdater.setTelemetryValue("Shooter Transfer Speed", shooterTransfer.getEncoder().getVelocity());
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
