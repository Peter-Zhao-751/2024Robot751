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
        this.transferCommand = new TransferCommand(300, transferSubsystem, TransferMode.Shoot, smartMode);
        this.speed = speed;
        this.hasShot = false;
        addRequirements(shooterSubsystem, transferSubsystem);
    }

    @Override
    public void initialize() {
        StateMachine.setState(StateMachine.State.Shoot);
        waitTime = 600;
        startTime = System.currentTimeMillis();
        shooterSubsystem.setSpeed(speed);
        hasShot = false;
        //transferCommand.initialize();
    }
    
    @Override
    public void execute() { // TODO: calibrate this
        if (!hasShot && System.currentTimeMillis() - startTime > waitTime){
            transferCommand.initialize();
            hasShot = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.setSpeed(0);
        if (transferCommand != null) transferCommand.end(interrupted);
        StateMachine.setState(StateMachine.State.Idle);
    }

    @Override
    public boolean isFinished() {
        return (transferCommand != null) ? transferCommand.isFinished() : false;
    }
}
