package frc.robot.commands.lowLevelCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.TransferSubsystem;

public class Transfer extends Command {
    private TransferSubsystem transferSubsystem;

    private double speed;
    private long startTime;
    private boolean smartMode;

    public Transfer(double speed, TransferSubsystem transferSubsystem, boolean smartMode) {
        this.speed = speed;
        this.transferSubsystem = transferSubsystem;
        this.smartMode = smartMode;
        addRequirements(transferSubsystem);
    }
    public Transfer(double speed, TransferSubsystem transferSubsystem) {
        this(speed, transferSubsystem, false);
    }

    public double getEstimatedTransferTime(){
        return Constants.Transfer.transferSystemLength / speed;
    }

    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        transferSubsystem.setIntakeTransfer(speed);
    }

    @Override
    public void end(boolean interrupted) {
        transferSubsystem.setIntakeTransfer(0);
    }

    @Override
    public boolean isFinished() {
        return transferSubsystem.beamBroken() || smartMode ? System.currentTimeMillis() - startTime >= Constants.Transfer.maxTransferTime * 1000 : false;
    }
}
