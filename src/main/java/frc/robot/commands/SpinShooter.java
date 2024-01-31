package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;

public class SpinShooter extends Command{
    private double startTime;
    private double totalTime;
    private double speed;
    private boolean shooting;
    
    public void initialize(boolean shooting, double speed, double totalTime) {
        super.initialize();
        this.shooting = shooting;
        this.speed = speed;
        this.totalTime = totalTime;
        this.startTime = System.currentTimeMillis();
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
