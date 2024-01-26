package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ShootCommand extends SequentialCommandGroup{
    public ShootCommand(Shooter shooter, PrimeShooter primeShooter) {
        addCommands(shooter, primeShooter);
    }
    
}
