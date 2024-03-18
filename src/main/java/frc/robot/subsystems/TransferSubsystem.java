package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.utility.TelemetryUpdater;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TransferSubsystem extends SubsystemBase implements Component {
    private static TransferSubsystem instance;

    private final CANSparkMax shooterTransfer;
    private final CANSparkMax intakeTransfer;
    private final DigitalInput beamBreak;

    private final PIDController shooterTransferPIDController;
    private final PIDController intakeTransferPIDController;

    private final Debouncer beamDebouncer;

    private double targetIntakeSpeed;
    private double targetShooterSpeed;
    private boolean isBeamBroken;

    private double allocatedCurrent;

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
        shooterTransfer.setInverted(false);
        beamBreak = new DigitalInput(Constants.Transfer.beamBreakDIOPort);
        beamDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kRising);

        shooterTransferPIDController = new PIDController(Constants.Transfer.kPIntakeController, 0, 0);
        intakeTransferPIDController = new PIDController(Constants.Transfer.kPShooterController, 0, 0);
        
        targetIntakeSpeed = 0;
        targetShooterSpeed = 0;

        isBeamBroken = false;

        allocatedCurrent = 0;
    }

    /**
     * Set the speed of the intake transfer motor
     * @param speed in centimeters per second
     */
    public void setIntakeTransfer(double speed) { // cm/s
        double targetIntakeSpeed = speed / (2 * Math.PI * Constants.Transfer.intakeTransferRadius) / 43;
        intakeTransfer.set(targetIntakeSpeed);
    }

    /**
     * Set the speed of the shooter transfer motor
     * @param speed in centimeters per second
     */
    public void setShooterTransfer(double speed) {
        //double targetShooterSpeed = speed / (2 * Math.PI * Constants.Transfer.shooterTransferRadius) / 43;
        //shooterTransfer.set(1);
        double targetShooterSpeed = Math.signum(speed);
        System.out.println("hfuidh");
        System.out.println("hfuidh");
        System.out.println("hfuidh");
        System.out.println("hfuidh");
        System.out.println(getShooterSpeed());
        System.out.println("hfuidh");
        shooterTransfer.set(targetShooterSpeed);
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
        //shooterTransfer.set(0);
        //intakeTransfer.set(0);
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
        TelemetryUpdater.setTelemetryValue("Beam Break", beamBroken());
        //TelemetryUpdater.setTelemetryValue("Transfer Current Draw", getCurrentDraw());

        TelemetryUpdater.setTelemetryValue("Intake Speed", getIntakeSpeed());
        TelemetryUpdater.setTelemetryValue("Shooter Speed", getShooterSpeed());
        TelemetryUpdater.setTelemetryValue("Intake Target Speed", targetIntakeSpeed);
        TelemetryUpdater.setTelemetryValue("Shooter Target Speed", targetShooterSpeed);

        // double currentIntakeSpeed = intakeTransfer.getEncoder().getVelocity() / 60;
        // double intakeOutput = intakeTransferPIDController.calculate(currentIntakeSpeed, targetIntakeSpeed);
        // TelemetryUpdater.setTelemetryValue("Intake Output", intakeOutput);
        

        // double currentShooterSpeed = shooterTransfer.getEncoder().getVelocity() / 60;
        // double shooterOutput = shooterTransferPIDController.calculate(currentShooterSpeed, targetShooterSpeed);
        // TelemetryUpdater.setTelemetryValue("Shooter Output", shooterOutput);
        //shooterTransfer.set(targetShooterSpeed);

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
