package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TransferSubsystem;

public class Transfer extends Command {
    private double speed;
    private boolean isFinished;
    private long startTime;
    private TransferSubsystem transferSubsystem;

    public Transfer(double speed, TransferSubsystem transferSubsystem) {
        this.speed = speed;
    }

    @Override
    public void initialize() {
        isFinished = false;
        startTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        if (System.currentTimeMillis() - startTime >= 3000 || transferSubsystem.beamBroken()) {
            transferSubsystem.setIntakeTransfer(0);
            isFinished = true;
        } else {
            transferSubsystem.setIntakeTransfer(speed);
        }
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
