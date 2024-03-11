package frc.robot.commands.lowLevelCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;
import com.ctre.phoenix6.hardware.Pigeon2;

public class ClimbCommand extends Command{
    private final ClimberSubsystem climberSubsystem;
    private final Pigeon2 gyro;
    
    public ClimbCommand(ClimberSubsystem climberSubsystem, Pigeon2 gyro){
        this.climberSubsystem = climberSubsystem;
        this.gyro = gyro;
        addRequirements(climberSubsystem);
    }
    @Override
    public void initialize() {
        // Climber code goes here
    }
    @Override
    public void execute() {
        // Climber code goes here
    }
    @Override
    public void end(boolean interrupted) {
        // Climber code goes here
    }
    @Override
    public boolean isFinished() {
        return false; // Ends the command after 3 seconds
    }
}
