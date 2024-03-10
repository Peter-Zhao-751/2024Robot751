package frc.robot.commands;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class AutoClimbCommand extends Command{
    private Pigeon2 pigeon;
    private ClimberSubsystem climberSubsystem;

    public AutoClimbCommand(ClimberSubsystem climberSubsystem, Pigeon2 pigeon){
        this.climberSubsystem = climberSubsystem;
        this.pigeon = pigeon;
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
