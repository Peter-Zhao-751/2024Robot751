package frc.robot.commands.movementCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utility.TelemetryUpdater;


public class AimAssistCommand extends Command {
	private final LimelightSubsystem limelight;
	private final SwerveSubsystem s_Swerve;

	private MoveCommand moveCommand;

	private double desiredAngle = 0;
	private Pose2d desiredPosition;

	private PIDController angleController;
	private PIDController distController;

	public AimAssistCommand() {
		this.limelight = LimelightSubsystem.getInstance();
		this.s_Swerve = SwerveSubsystem.getInstance();

		this.angleController = new PIDController(0.1, 0, 0);
		this.distController = new PIDController(0, 0, 0);

		// TelemetryUpdater.setTelemetryValue("autoaim/angle/kp", 0.0);
		TelemetryUpdater.setTelemetryValue("autoaim/angle/ki", 0.0);
		TelemetryUpdater.setTelemetryValue("autoaim/angle/kd", 0.0);

		addRequirements(s_Swerve);
	}

	@Override
	public void initialize() {
		Pose2d pose = limelight.getPose();
		limelight.setVisionMode();

		// if there is no target, then cancel the command
		if (pose == null) {
			System.err.println("No target found");
			// cancel();
			return;
		}

		// if the 2 blue field element is closer than the 2 red, then use the blue field elements, otherwise use the red field elements
		FieldConstants.FieldElements redSpeaker = FieldConstants.red[1];
		FieldConstants.FieldElements blueSpeaker = FieldConstants.blue[1];

		// FieldConstants.FieldElements closestSpeaker = Math.hypot(redSpeaker.x - redSpeaker.y, pose.getX() - pose.getY()) <
		// 		Math.hypot(blueSpeaker.x - blueSpeaker.y, pose.getX() - pose.getY()) ? redSpeaker : blueSpeaker;
		// TODO: Maybe change to an id based system using LimelightHelpers.getFiducialID() instead of this
		FieldConstants.FieldElements closestSpeaker = redSpeaker;
		TelemetryUpdater.setTelemetryValue("aimbot/pose/x", pose.getX());
		TelemetryUpdater.setTelemetryValue("aimbot/pose/y", pose.getY());
		TelemetryUpdater.setTelemetryValue("aimbot/speaker/x", closestSpeaker.x);
		TelemetryUpdater.setTelemetryValue("aimbot/speaker/y", closestSpeaker.y);
		
		// speaker: 15.76, 5.28
		// us: 12.5, 6
        double angle = (Math.toDegrees(Math.atan2(closestSpeaker.y - pose.getY(), closestSpeaker.x - pose.getX())) + 360) % 360;
		angle = angle > 180 ? angle - 360: angle;
		// angle - pose2d.getRotation().getDegrees() + 360) % 360;
		double delta = limelight.getYaw() - angle;

		desiredAngle = (s_Swerve.getGyroYaw().getDegrees() - delta) % 360;
		TelemetryUpdater.setTelemetryValue("aimbot/speakerAngle", angle);
		TelemetryUpdater.setTelemetryValue("aimbot/limelightYaw", -limelight.getYaw());
		TelemetryUpdater.setTelemetryValue("aimbot/delta", delta);

		// Pose2d pose2d = s_Swerve.getSwerveOdometryPose2d();

		double desired = Units.feetToMeters(10);
		double distance = Math.hypot(closestSpeaker.x - pose.getX(), closestSpeaker.y - pose.getY());
		desiredPosition = s_Swerve.getPose().plus(new Transform2d(new Translation2d(0, desired - distance), Rotation2d.fromDegrees(angle)));

		TelemetryUpdater.setTelemetryValue("autoaim/dist/distance", distance);
		TelemetryUpdater.setTelemetryValue("autoaim/dist/x", desiredPosition.getX());
		TelemetryUpdater.setTelemetryValue("autoaim/dist/y", desiredPosition.getY());
	}

	@Override
    public void execute() {
		double rot = angleController.calculate(s_Swerve.getGyroYaw().getDegrees(), desiredAngle);
		rot = Math.min(rot, 1);
		TelemetryUpdater.setTelemetryValue("autoaim/angle/voltage", rot);
		TelemetryUpdater.setTelemetryValue("autoaim/angle/desired", desiredAngle);
		// angleController.setP((Double) TelemetryUpdater.getTelemetryValue("autoaim/angle/kp"));
		// angleController.setI((Double) TelemetryUpdater.getTelemetryValue("autoaim/angle/ki"));
		// angleController.setD((Double) TelemetryUpdater.getTelemetryValue("autoaim/angle/kd"));

		angleController.setP(SmartDashboard.getNumber("autoaim/angle/kp", 0.1));
		s_Swerve.drive(new Translation2d(), rot , false, true, false, false);
    }

    @Override
    public void end(boolean interrupted) {
		
	}

    @Override
    public boolean isFinished() {
        return false;
    }
}
