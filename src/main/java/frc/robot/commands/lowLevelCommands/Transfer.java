package frc.robot.commands.lowLevelCommands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.TransferSubsystem;

public class Transfer extends Command {

    public enum TransferMode {
        Intake,
        Outtake,
        Shoot;
    }

    private TransferSubsystem transferSubsystem;
    Debouncer beamDebouncer;
    private double speed;
    private long startTime;
    private TransferMode transferMode;
    private boolean smartMode;
    private boolean isBeamBroken;

    public Transfer(double speed, TransferSubsystem transferSubsystem, TransferMode transferMode, boolean smartMode) {
        this.beamDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kBoth);
        this.speed = (transferMode == TransferMode.Outtake) ? - speed : speed;
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
    }

    @Override
    public void execute() {
        isBeamBroken = beamDebouncer.calculate(transferSubsystem.beamBroken());
    }

    @Override
    public void end(boolean interrupted) {
        transferSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        double timeDelta = System.currentTimeMillis() - startTime;
        boolean smartBeamBreak = isBeamBroken && timeDelta > Constants.Transfer.minTransferTime;
        if (transferMode == TransferMode.Intake) {
            if (smartMode) {
                return smartBeamBreak || timeDelta > Constants.Transfer.maxTransferTime;
            } else {
                return smartBeamBreak;
            }
        } else {
            if (smartMode) {
                return (!isBeamBroken && timeDelta > Constants.Transfer.minTransferTime) || timeDelta > Constants.Transfer.maxTransferTime;
            } else {
                return false;
            }
        }
    }
}
