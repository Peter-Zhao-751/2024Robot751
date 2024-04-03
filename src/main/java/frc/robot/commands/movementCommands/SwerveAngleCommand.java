package frc.robot.commands.movementCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;


public class SwerveAngleCommand extends Command {
    private final SwerveSubsystem swerveSubsystem;
    public enum SwerveAngle {
       CROSS,
       ZERO
    }

    private final SwerveAngle swerveAngle;

    public SwerveAngleCommand(SwerveAngle swerveAngle) {
       this.swerveSubsystem = SwerveSubsystem.getInstance();
       this.swerveAngle = swerveAngle;

       addRequirements(this.swerveSubsystem);
    }

    @Override
    public void initialize() {
        switch (swerveAngle) {
            case ZERO -> swerveSubsystem.resetModulesToAbsolute();
            case CROSS -> swerveSubsystem.crossWheels();
        }
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}
