package frc.robot.commands.movementCommands;

import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class MoveCommand extends Command {
    private final SwerveSubsystem s_Swerve;
    private final Pose2d desiredLocation;
    private final List<Translation2d> interiorWaypoints;
    private double ETA;
	private SwerveControllerCommand swerveControllerCommand;
	private SwerveRotateControllerCommand swerveRotateControllerCommand;

    public MoveCommand(Pose2d desiredLocation, List<Translation2d> interiorWaypoints) {
		this.desiredLocation = desiredLocation;
        this.interiorWaypoints = interiorWaypoints;
		this.s_Swerve = SwerveSubsystem.getInstance();
        addRequirements(s_Swerve);
    }

    public MoveCommand(Pose2d desiredLocation) {
        this(desiredLocation, List.of());
        addRequirements(s_Swerve);
    }
    @Override
    public void initialize() {
        Pose2d currentRobotPosition = s_Swerve.getPose();
		s_Swerve.setSwerveOdometryPose2d(currentRobotPosition);

		if (Math.abs(currentRobotPosition.getTranslation().getDistance(desiredLocation.getTranslation())) < 0.1 && Math.abs(currentRobotPosition.getRotation().getDegrees() - desiredLocation.getRotation().getDegrees()) < 5) return;

		if (Math.abs(currentRobotPosition.getTranslation().getDistance(desiredLocation.getTranslation())) < 1) {

			swerveRotateControllerCommand = new SwerveRotateControllerCommand(desiredLocation.getRotation());
			swerveRotateControllerCommand.initialize();
			ETA = 1;
		} else {
			TrajectoryConfig config = new TrajectoryConfig(
					Constants.AutoConstants.kMaxSpeedMetersPerSecond,
					Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
					.setKinematics(Constants.Swerve.swerveKinematics);

			Trajectory movementTrajectory = TrajectoryGenerator.generateTrajectory(
					currentRobotPosition,
					interiorWaypoints,
					desiredLocation,
					config);

			ProfiledPIDController thetaController = new ProfiledPIDController(
					Constants.AutoConstants.kPThetaController, 0, 0,
					Constants.AutoConstants.kThetaControllerConstraints);
			thetaController.enableContinuousInput(-Math.PI, Math.PI);

			ETA = movementTrajectory.getTotalTimeSeconds();
			//TelemetryUpdater.setTelemetryValue("Auton Current Trajectory Estimated ETA", ETA);

			swerveControllerCommand = new SwerveControllerCommand(
					movementTrajectory,
					s_Swerve::getSwerveOdometryPose2d, // i might be retardes
					Constants.Swerve.swerveKinematics,
					new PIDController(Constants.AutoConstants.kPXController, 0, 0),
					new PIDController(Constants.AutoConstants.kPYController, 0, 0),
					thetaController,
					s_Swerve::setModuleStates,
					s_Swerve);

			swerveControllerCommand.initialize();
		}
    }

    @Override
    public void execute() {
        if (swerveControllerCommand != null) {
            swerveControllerCommand.execute();
        } else if (swerveRotateControllerCommand != null) {
			swerveRotateControllerCommand.execute();
		}
    }

    @Override
    public void end(boolean interrupted) {
        if (swerveControllerCommand != null) {
            swerveControllerCommand.end(interrupted);
		} else if (swerveRotateControllerCommand != null) {
			swerveRotateControllerCommand.end(interrupted);
		}
    }

    @Override
    public boolean isFinished() {
		if (swerveControllerCommand != null) {
			return swerveControllerCommand.isFinished();
		} else if (swerveRotateControllerCommand != null) {
			return swerveRotateControllerCommand.isFinished();
		}
		return true;
    }

    public double getETA() {
        return ETA;
    }
}