package frc.robot.commands.lowLevelCommands;
import frc.robot.Constants;
import frc.robot.commands.lowLevelCommands.TransferCommand.TransferMode;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.utility.StateMachine;
import edu.wpi.first.wpilibj2.command.Command;

public class ShootCommand extends Command{
    private final ShooterSubsystem shooterSubsystem;
    private final TransferSubsystem transferSubsystem;
    private final TransferCommand transferCommand;
    private double speed;
    private boolean smartMode;
 
    public ShootCommand(ShooterSubsystem shooterSubsystem, TransferSubsystem transferSubsystem, double speed, boolean smartMode) {
        this.shooterSubsystem = shooterSubsystem;
        this.transferSubsystem = transferSubsystem;
        this.transferCommand = new TransferCommand(Constants.Transfer.feedSpeed, transferSubsystem, TransferMode.Shoot, smartMode);
        addRequirements(shooterSubsystem, transferSubsystem);
    }

    @Override
    public void initialize() {
        StateMachine.setState(StateMachine.State.Shoot);
        shooterSubsystem.setSpeed(20);
        transferCommand.initialize();
    }
    
    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {

        shooterSubsystem.setSpeed(0);
        System.out.println("i hate this team");
        if (transferCommand != null) transferCommand.end(interrupted);
        StateMachine.setState(StateMachine.State.Idle);
        // doesnt need its own end method because transfer.end() will be called in the end method of this command
    }

    @Override
    public boolean isFinished() {
        return transferCommand.isFinished();
    }
}
