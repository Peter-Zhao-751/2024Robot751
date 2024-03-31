gitpackage frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.utility.FeedforwardPID;
import frc.robot.utility.TelemetryUpdater;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TransferSubsystem extends SubsystemBase {
    private static TransferSubsystem instance;

    private final CANSparkMax shooterTransfer;
    private final CANSparkMax intakeTransfer;

    private final FeedforwardPID shooterTransferController;
    private final FeedforwardPID intakeTransferController;

    private final DigitalInput beamBreak;
    private final Debouncer beamDebouncer;
    private boolean isBeamBroken;

    private double targetIntakeSpeed;
    private double targetShooterSpeed;

    public static TransferSubsystem getInstance() {
        if (instance == null) {
            instance = new TransferSubsystem();
        }
        return instance;
    }

    private TransferSubsystem() {
        shooterTransfer = new CANSparkMax(Constants.Transfer.shooterTransferID, MotorType.kBrushless);
        intakeTransfer = new CANSparkMax(Constants.Transfer.intakeTransferID, MotorType.kBrushless);
        intakeTransfer.setInverted(true);
        shooterTransfer.setInverted(true);

        beamBreak = new DigitalInput(Constants.Transfer.beamBreakDIOPort);
        beamDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kRising);
        isBeamBroken = false;

        shooterTransferController = new FeedforwardPID(0, 0, 0, Constants.Transfer.kPShooterController, 0, 0);
        intakeTransferController = new FeedforwardPID(0, 0, 0, Constants.Transfer.kPIntakeController, 0, 0);
        
        targetIntakeSpeed = 0;
        targetShooterSpeed = 0;
    }

    /**
     * Set the speed of the intake transfer motor
     * @param speed in centimeters per second
     */
    public void setIntakeTransfer(double speed) { // cm/s
//        targetIntakeSpeed = speed / (2 * Math.PI * Constants.Transfer.intakeTransferRadius);
        intakeTransfer.set(speed/40);
    }

    /**
     * Set the speed of the shooter transfer motor
     * @param speed in centimeters per second
     */
    public void setShooterTransfer(double speed) {
//        targetShooterSpeed = speed / (2 * Math.PI * Constants.Transfer.shooterTransferRadius);
        shooterTransfer.set(speed/40);
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
//        targetIntakeSpeed = 0;
//        targetShooterSpeed = 0;
        intakeTransfer.stopMotor();
        shooterTransfer.stopMotor();
    }

    /**
     * Returns if the beam is broken
     * @return boolean, true if the beam is broken
     */
    public boolean beamBroken() { // True when beam is broken
        return isBeamBroken;
    }

    public double getIntakeSpeed() { // rps
        return intakeTransfer.getEncoder().getVelocity() / 60;
    }

    public double getShooterSpeed() { // rps
        return shooterTransfer.getEncoder().getVelocity() / 60;
    }


    @Override
    public void periodic() {
        isBeamBroken = beamDebouncer.calculate(!beamBreak.get());
        TelemetryUpdater.setTelemetryValue("Transfer Beam Break", isBeamBroken);

        TelemetryUpdater.setTelemetryValue("Intake Transfer Speed", getIntakeSpeed());
        TelemetryUpdater.setTelemetryValue("Shooter Transfer Speed", getShooterSpeed());
        TelemetryUpdater.setTelemetryValue("Intake Transfer Target Speed", targetIntakeSpeed);
        TelemetryUpdater.setTelemetryValue("Shooter Transfer Target Speed", targetShooterSpeed);

        // intakeTransfer.setVoltage(intakeTransferController.calculate(getIntakeSpeed(), targetIntakeSpeed));
        // shooterTransfer.setVoltage(shooterTransferController.calculate(getShooterSpeed(), targetShooterSpeed));

        intakeTransferController.debugPrintouts("intake transfer");
        shooterTransferController.debugPrintouts("shooter transfer");
    }
}
