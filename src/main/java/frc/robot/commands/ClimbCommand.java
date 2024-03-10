package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimbCommand extends Command{
    private ClimberSubsystem climberSubsystem;
    
    public ClimbCommand(ClimberSubsystem climberSubsystem){
        this.climberSubsystem = climberSubsystem;
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
