package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utility.StateMachine;


public class StupidAimAssistCommand extends Command {
	private final LimelightSubsystem limelight;
	private final SwerveSubsystem s_Swerve;

	private MoveCommand moveCommand;

	public StupidAimAssistCommand() {
		this.limelight = LimelightSubsystem.getInstance();
		this.s_Swerve = SwerveSubsystem.getInstance();
		addRequirements(s_Swerve);
	}

	@Override
	public void initialize() {
		StateMachine.setState(StateMachine.State.Aimbot);
		limelight.setVisionMode();
		limelight.setLEDMode(LimelightSubsystem.LEDMode.ON);
		Pose2d pose = limelight.getPose();

		// if there is no target, then cancel the command
		if (pose == null) {
			cancel();
			System.err.println("No target found");
			return;
		}

		// if the 2 blue field element is closer than the 2 red, then use the blue field elements, otherwise use the red field elements
		FieldConstants.FieldElements redSpeaker = FieldConstants.red[1];
		FieldConstants.FieldElements blueSpeaker = FieldConstants.blue[1];

		FieldConstants.FieldElements closestSpeaker = getDistance(redSpeaker.x, redSpeaker.y, pose.getX(), pose.getY()) <
				getDistance(blueSpeaker.x, blueSpeaker.y, pose.getX(), pose.getY()) ? redSpeaker : blueSpeaker;
		// TODO: Maybe change to an id based system using LimelightHelpers.getFiducialID() instead of this


        double angle = Math.atan2(closestSpeaker.y - pose.getY(), closestSpeaker.x - pose.getX());

		angle = Math.toDegrees(angle);
		angle = (angle + 360) % 360;

		moveCommand = new MoveCommand(new Pose2d(pose.getX(), pose.getY(), new Rotation2d(angle)));
		moveCommand.initialize();
	}

	private double getDistance(double x1, double y1, double x2, double y2) {
		return Math.sqrt(Math.pow(x1 - x2, 2) + Math.pow(y1 - y2, 2));
	}

	@Override
    public void execute() {
		if (moveCommand != null) moveCommand.execute();
    }

    @Override
    public void end(boolean interrupted) {
        StateMachine.setState(StateMachine.State.Idle);
		limelight.setDriverMode();
		limelight.setLEDMode(LimelightSubsystem.LEDMode.OFF);
    }

    @Override
    public boolean isFinished() {
        return moveCommand == null || moveCommand.isFinished();
    }
}
