package frc.robot.commands.movementCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;


public class AimAssistCommand extends Command {
	private final LimelightSubsystem limelight;
	private final SwerveSubsystem s_Swerve;

	public AimAssistCommand() {
		this.limelight = LimelightSubsystem.getInstance();
		this.s_Swerve = SwerveSubsystem.getInstance();
	}

	@Override
	public void initialize() {
		limelight.setVisionMode();
		limelight.setLEDMode(LimelightSubsystem.LEDMode.ON);
		s_Swerve.lockOnTarget = true;
	}

    @Override
    public void end(boolean interrupted) {
		limelight.setLEDMode(LimelightSubsystem.LEDMode.OFF);
		s_Swerve.lockOnTarget = false;
	}
}
