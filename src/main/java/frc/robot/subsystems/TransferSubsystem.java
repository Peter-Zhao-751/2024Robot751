package frc.robot.subsystems;

import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TransferSubsystem extends SubsystemBase implements Component {
    private final CANSparkMax shooterTransfer;
    private final CANSparkMax intakeTransfer;
    private final DigitalInput beamBreak;

    private final double shooterTransferRadius = Constants.Transfer.shooterTransferRadius;
    private final double intakeTransferRadius = Constants.Transfer.intakeTransferRadius;

    public TransferSubsystem() {
        shooterTransfer = new CANSparkMax(Constants.Transfer.shooterTransferID, MotorType.kBrushless);
        intakeTransfer = new CANSparkMax(Constants.Transfer.intakeTransferID, MotorType.kBrushless);
        beamBreak = new DigitalInput(Constants.Transfer.beamBreakID);
    }

    public void setIntakeTransfer(double speed) {
        intakeTransfer.set(speed);
    }

    public void setShooterTransfer(double speed) {
        shooterTransfer.set(speed);
    }

    public void transfer(double speed) { // in inches per second of surface speed
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
    public double getRequestedCurrent() {
        return shooterTransfer.getOutputCurrent() + intakeTransfer.getOutputCurrent();
    }

    @Override
    public void allocateCurrent(double current) {
    }

    @Override
    public int getPriority() {
        return 1; // CHANGE
    }

    @Override
    public void updateBasedOnAllocatedCurrent() {}
}
