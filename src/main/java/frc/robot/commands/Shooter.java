package frc.robot.commands;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Command;

public class Shooter extends Command{
    private ShooterSubsystem shooterSubsystem;
    private TransferSubsystem transferSubsystem;
    private double startTime;
    private double speed;
    private double totalRunTime;
    public Shooter(ShooterSubsystem shooterSubsystem, TransferSubsystem transferSubsystem, double speed) {
        this.shooterSubsystem = shooterSubsystem;
        this.transferSubsystem = transferSubsystem;
        this.speed = speed;
        addRequirements(shooterSubsystem);
    }
    public Shooter(ShooterSubsystem shooterSubsystem, TransferSubsystem transferSubsystem) {
        this(shooterSubsystem, transferSubsystem, 0.75);
    }
    @Override
    public void initialize() {
        double currentSpeed = shooterSubsystem.getShooterSpeed();

        double ratio = 1 - currentSpeed / speed;
        totalRunTime = ratio * Constants.Shooter.spinUpTime * 1000 + Constants.Shooter.feedTime * 1000 + 500; // magic number is for safety

        startTime = System.currentTimeMillis();
        shooterSubsystem.shoot(speed);
    }

    @Override
    public void execute() {
        if (Math.abs(shooterSubsystem.getShooterSpeed() - speed) <= 0.05 || System.currentTimeMillis() - startTime <= Constants.Shooter.feedTime * 1000) {
            transferSubsystem.transfer(Constants.Shooter.transferSpeed);
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.stop();
    }
    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() - startTime >= totalRunTime;
    }
}
