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
    private double speed;
    private long startTime;
    private final TransferMode transferMode;
    private boolean smartMode;

    public TransferCommand(double speed, TransferSubsystem transferSubsystem, TransferMode transferMode, boolean smartMode) {
        this.speed = (TransferMode.Intake == transferMode) ? speed : -speed;

        this.transferSubsystem = transferSubsystem;
        this.transferMode = transferMode;
        this.smartMode = smartMode;

        addRequirements(transferSubsystem);
    }

    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();
        transferSubsystem.setIntakeTransfer(speed);
        transferSubsystem.setShooterTransfer(speed);
    }

    @Override
    public void execute() {
        
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
        boolean smartBeamBreak = transferSubsystem.beamBroken() && overMinTime;
        if (transferMode == TransferMode.Intake) {
            if (smartMode) {
                return smartBeamBreak || overMaxTime;
            } else {
                return smartBeamBreak;
            }
        } else {
            if (smartMode) {
                return (!transferSubsystem.beamBroken() && overMinTime) || overMaxTime;
            } else {
                return false;
            }
        }
    }
}
