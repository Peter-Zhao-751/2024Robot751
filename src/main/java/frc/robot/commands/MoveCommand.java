package frc.robot.commands;

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

    public MoveCommand(SwerveSubsystem s_Swerve, Pose2d desiredLocation, List<Translation2d> interiorWaypoints) {
        this.s_Swerve = s_Swerve;
        this.desiredLocation = desiredLocation;
        this.interiorWaypoints = interiorWaypoints;
        this.movementTrajectory = null;
        addRequirements(s_Swerve);
    }

    public MoveCommand(SwerveSubsystem s_Swerve, Pose2d desiredLocation) {
        this(s_Swerve, desiredLocation, List.of());
        addRequirements(s_Swerve);
    }

    public MoveCommand(SwerveSubsystem s_Swerve, Trajectory trajectory) {
        this.s_Swerve = s_Swerve;
        this.desiredLocation = trajectory.getStates().get(trajectory.getStates().size() - 1).poseMeters;
        this.interiorWaypoints = null;
        this.movementTrajectory = trajectory;
        addRequirements(s_Swerve);
    }


    @Override
    public void initialize() {
        Pose2d currentRobotPosition = s_Swerve.getSwerveOdometryPose2d(); // do something

        if (Math.abs(desiredLocation.getX() - currentRobotPosition.getX()) < 0.1 && Math.abs(desiredLocation.getY() - currentRobotPosition.getY()) < 0.1) return;
        
        if (movementTrajectory == null){
            TrajectoryConfig config = new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics);

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

        ETA = movementTrajectory.getTotalTimeSeconds();
        //TelemetryUpdater.setTelemetryValue("Auton Current Trajectory Estimated ETA", ETA);

        swerveControllerCommand = new SwerveControllerCommand(
                movementTrajectory,
                s_Swerve::getSwerveOdometryPose2d,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);
        
        swerveControllerCommand.initialize();
    }

    @Override
    public void execute() {
        if (swerveControllerCommand != null) {
            swerveControllerCommand.execute();
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (swerveControllerCommand != null) {
            swerveControllerCommand.end(interrupted);
        }
    }

    @Override
    public boolean isFinished() {
        return swerveControllerCommand != null && swerveControllerCommand.isFinished();
    }

    public double getETA() {
        return ETA;
    }
}