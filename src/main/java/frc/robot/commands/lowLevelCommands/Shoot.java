package frc.robot.commands.lowLevelCommands;
import frc.robot.Constants;
import frc.robot.commands.lowLevelCommands.Transfer.TransferMode;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class Shoot extends Command{
    private ShooterSubsystem shooterSubsystem;
    private TransferSubsystem transferSubsystem;
    private Transfer transfer;
    private double speed;
    private boolean smartMode;
 
    public Shoot(ShooterSubsystem shooterSubsystem, TransferSubsystem transferSubsystem, double speed, boolean smartMode) {
        this.shooterSubsystem = shooterSubsystem;
        this.transferSubsystem = transferSubsystem;
        this.transfer = new Transfer(Constants.Transfer.feedSpeed, transferSubsystem, TransferMode.Shoot, smartMode);
        addRequirements(shooterSubsystem, transferSubsystem);
    }

    @Override
    public void initialize() {
        shooterSubsystem.setSpeed(speed);
        transfer.initialize();
    }
    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.stop();
        // doesnt need its own end method because transfer.end() will be called in the end method of this command
    }

    @Override
    public boolean isFinished() {
        return transfer.isFinished();
    }
}
