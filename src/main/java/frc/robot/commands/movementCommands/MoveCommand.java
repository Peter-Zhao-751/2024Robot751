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
    private Trajectory movementTrajectory;
    private SwerveControllerCommand swerveControllerCommand;

    public MoveCommand(Pose2d desiredLocation, List<Translation2d> interiorWaypoints) {
        this.s_Swerve = SwerveSubsystem.getInstance();
        this.desiredLocation = desiredLocation;
        this.interiorWaypoints = interiorWaypoints;
        this.movementTrajectory = null;
        addRequirements(s_Swerve);
    }

    public MoveCommand(Pose2d desiredLocation) {
        this(desiredLocation, List.of());
    }

    public MoveCommand(Trajectory trajectory) {
        this.s_Swerve = SwerveSubsystem.getInstance();
        this.desiredLocation = trajectory.getStates().get(trajectory.getStates().size() - 1).poseMeters;
        this.interiorWaypoints = null;
        this.movementTrajectory = trajectory;
        addRequirements(s_Swerve);
    }

    @Override
    public void initialize() { // TODO: make stuff use getpose which gets the kalman pose once testing is complete
        Pose2d currentRobotPosition = s_Swerve.getPose(); // do something

        if (isAtDesiredLocation(currentRobotPosition, desiredLocation, interiorWaypoints) &&
            Math.abs(desiredLocation.getRotation().getDegrees() - currentRobotPosition.getRotation().getDegrees()) < 5) return;

        if (movementTrajectory == null && !isAtDesiredLocation(currentRobotPosition, desiredLocation, interiorWaypoints)) {
            TrajectoryConfig config = new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics);

            //config.setStartVelocity(s_Swerve.getCurrentVelocityMagnitude());

            movementTrajectory = TrajectoryGenerator.generateTrajectory(
                currentRobotPosition,
                interiorWaypoints,
                desiredLocation,
                config);
        }

        ProfiledPIDController thetaController = new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0,
				Constants.AutoConstants.kThetaControllerConstraints);

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        if (movementTrajectory != null) ETA = movementTrajectory.getTotalTimeSeconds();
        //TelemetryUpdater.setTelemetryValue("Auton Current Trajectory Estimated ETA", ETA);

        swerveControllerCommand = new SwerveControllerCommand(
                movementTrajectory,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
				s_Swerve);

        swerveControllerCommand.initialize();
    }

    @Override
    public void end(boolean interrupted) {
        if (swerveControllerCommand != null) swerveControllerCommand.end(interrupted);
    }

    @Override
    public void execute(){
        if (swerveControllerCommand != null) swerveControllerCommand.execute();
    }

    @Override
    public boolean isFinished() {
        return swerveControllerCommand != null && swerveControllerCommand.isFinished();
    }

	public double getETA() {
		return ETA;
	}

	private boolean isAtDesiredLocation(Pose2d currPose2d, Pose2d desiredPose2d, List<Translation2d> interiorWaypoints) {
		return Math.abs(desiredPose2d.getX() - currPose2d.getX()) < 0.1 && Math.abs(desiredPose2d.getY() - currPose2d.getY()) < 0.1 && interiorWaypoints.isEmpty();
	}
}