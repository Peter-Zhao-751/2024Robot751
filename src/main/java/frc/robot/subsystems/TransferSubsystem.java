package frc.robot.subsystems;

import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TransferSubsystem extends SubsystemBase implements Component {
    private final CANSparkMax shooterTransfer;
    private final CANSparkMax intakeTransfer;
    private final DigitalInput beamBreak;

    private final double shooterTransferRadius = Constants.Transfer.shooterTransferRadius;
    private final double intakeTransferRadius = Constants.Transfer.intakeTransferRadius;

    private final PIDController shooterTransferPIDController;
    private final PIDController intakeTransferPIDController;

    private double currentDraw;
    private double allocatedCurrent;

    public TransferSubsystem() {
        shooterTransfer = new CANSparkMax(Constants.Transfer.shooterTransferID, MotorType.kBrushless);
        intakeTransfer = new CANSparkMax(Constants.Transfer.intakeTransferID, MotorType.kBrushless);
        beamBreak = new DigitalInput(Constants.Transfer.beamBreakID);

        shooterTransferPIDController = new PIDController(Constants.Transfer.kPIntakeController, 0, 0);
        intakeTransferPIDController = new PIDController(Constants.Transfer.kPShooterController, 0, 0);

        currentDraw = 0;
        allocatedCurrent = 0;
    }

    public void setIntakeTransfer(double speed) { // in rps
        double currentSpeed = intakeTransfer.getEncoder().getVelocity() / 60;
        double output = intakeTransferPIDController.calculate(currentSpeed, speed);
        intakeTransfer.set(output);
    }

    public void setShooterTransfer(double speed) { // in rps
        double currentSpeed = shooterTransfer.getEncoder().getVelocity() / 60;
        double output = shooterTransferPIDController.calculate(currentSpeed, speed);
        shooterTransfer.set(output);
    }

    public void transfer(double speed) { // in centimeters per second of surface speed
        setIntakeTransfer(speed / (2 * Math.PI * intakeTransferRadius));
        setShooterTransfer(speed / (2 * Math.PI * shooterTransferRadius));
    }

    public boolean beamBroken() { // True when beam is broken
        return !beamBreak.get();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Beam Break", beamBroken());
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
