package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;

public class PrimeShooter extends Command{
    private double startTime;
    private double totalTime;
    @Override
    public void initialize() {
        
        startTime = System.currentTimeMillis();
        //motorSubsystem.runMotor(speed);
    }

    @Override
    public void execute() {
        // The actual motor control is handled in the subsystem
    }

    @Override
    public void end(boolean interrupted) {
        //motorSubsystem.stopMotor(); // Stop the motor when the command ends
    }
    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() - startTime >= totalTime; // Ends the command after 3 seconds
    }
}
