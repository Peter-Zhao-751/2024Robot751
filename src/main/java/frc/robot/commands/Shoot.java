package frc.robot.commands;
import frc.robot.Constants;
import frc.robot.commands.lowLevelCommands.Transfer;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Command;

public class Shoot extends Command{
    private ShooterSubsystem shooterSubsystem;
    private TransferSubsystem transferSubsystem;
    private Transfer transfer;
    private double speed;
    private double shootWaitTime;
    private boolean hasStartedShooter;
    private double startTime;
 
    public Shoot(ShooterSubsystem shooterSubsystem, TransferSubsystem transferSubsystem, double speed) {
        this.shooterSubsystem = shooterSubsystem;
        this.transferSubsystem = transferSubsystem;
        addRequirements(shooterSubsystem, transferSubsystem);
        hasStartedShooter = false;
    }
    public Shoot(ShooterSubsystem shooterSubsystem, TransferSubsystem transferSubsystem) {
        this(shooterSubsystem, transferSubsystem, 0.75);
    }
    @Override
    public void initialize() {
        transfer = new Transfer(Constants.Transfer.feedSpeed, transferSubsystem);
        shootWaitTime = transfer.getEstimatedTransferTime() - shooterSubsystem.getEstimatedSpinUpTime(speed);
        startTime = System.currentTimeMillis();
        transfer.initialize();
    }
    @Override
    public void execute() {
        if(System.currentTimeMillis() - startTime >= shootWaitTime && !hasStartedShooter){
            shooterSubsystem.setSpeed(speed);
            hasStartedShooter = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.stop();
        transfer.end(interrupted);
    }
    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() - startTime >= 10000; // TODO: fix this
    }
}
