package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.TransferSubsystem;

public class Transfer extends Command {
    private double speed;
    private long startTime;
    private TransferSubsystem transferSubsystem;

    public Transfer(double speed, TransferSubsystem transferSubsystem) {
        this.speed = speed;
        this.transferSubsystem = transferSubsystem;
        addRequirements(transferSubsystem);
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
        return System.currentTimeMillis() - startTime >= Constants.Transfer.maxTransferTime * 1000 || transferSubsystem.beamBroken();
    }
}
