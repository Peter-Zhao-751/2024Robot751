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
    private double startTime;
    private double waitTime;
    private boolean hasShot;
 
    public ShootCommand(ShooterSubsystem shooterSubsystem, TransferSubsystem transferSubsystem, double speed, boolean smartMode) {
        this.shooterSubsystem = shooterSubsystem;
        this.transferSubsystem = transferSubsystem;
        this.transferCommand = new TransferCommand(Constants.Transfer.feedSpeed, transferSubsystem, TransferMode.Shoot, smartMode);
        this.speed = speed;
        this.hasShot = false;
        addRequirements(shooterSubsystem, transferSubsystem);
    }

    @Override
    public void initialize() {
        StateMachine.setState(StateMachine.State.Shoot);
        waitTime = Math.max(0, shooterSubsystem.getTargetETA() - Constants.Transfer.minTransferTime);
        startTime = System.currentTimeMillis();
        shooterSubsystem.setSpeed(speed);
        //transferCommand.initialize();
    }
    
    @Override
    public void execute() { // TODO: calibrate this
        if (!hasShot && System.currentTimeMillis() - startTime > waitTime){
            transferCommand.schedule();
            hasShot = true;
        }
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
        return (transferCommand != null) ? transferCommand.isFinished() : false;
    }
}
