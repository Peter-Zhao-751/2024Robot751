package frc.robot.commands.lowLevelCommands;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.utility.TelemetryUpdater;

public class TransferCommand extends Command {

    public enum TransferMode {
        Intake,
        Outtake,
        Shoot;
    }

    private final TransferSubsystem transferSubsystem;
    Debouncer beamDebouncer;
    private double speed;
    private long startTime;
    private final TransferMode transferMode;
    private boolean smartMode;
    private boolean isBeamBroken;

    public TransferCommand(double speed, TransferSubsystem transferSubsystem, TransferMode transferMode, boolean smartMode) {
        this.beamDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kBoth);
        this.speed = (TransferMode.Intake == transferMode) ? speed : -speed;

        this.transferSubsystem = transferSubsystem;
        this.transferMode = transferMode;
        this.smartMode = smartMode;
        this.isBeamBroken = false;

        addRequirements(transferSubsystem);
    }

    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();
        transferSubsystem.setIntakeTransfer(speed);
        if (transferMode == TransferMode.Shoot) transferSubsystem.setShooterTransfer(speed);
    }

    @Override
    public void execute() {
        isBeamBroken = beamDebouncer.calculate(transferSubsystem.beamBroken());
        TelemetryUpdater.setTelemetryValue("Transfer Beam Broken", isBeamBroken);
    }

    @Override
    public void end(boolean interrupted) {
        transferSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        // TODO: this part is shit 
        //return false;
        double timeDelta = System.currentTimeMillis() - startTime;
        boolean overMinTime = timeDelta > Constants.Transfer.minTransferTime * 1000;
        boolean overMaxTime = timeDelta > Constants.Transfer.maxTransferTime * 1000;
        boolean smartBeamBreak = isBeamBroken && overMinTime;
        if (transferMode == TransferMode.Intake) {
            if (smartMode) {
                return smartBeamBreak || overMaxTime;
            } else {
                return smartBeamBreak;
            }
        } else {
            if (smartMode) {
                return (!isBeamBroken && overMinTime) || overMaxTime;
            } else {
                return false;
            }
        }
    }
}
