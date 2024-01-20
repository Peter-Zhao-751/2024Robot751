package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class MoveToLocation extends SequentialCommandGroup {
    private double ETA = 0;
    public MoveToLocation(SwerveDrive s_Swerve, Pose2d desiredLocation, List<Translation2d> interiorWaypoints){
        TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics);

        Pose2d currentRobotPosition = s_Swerve.getPose();
        Trajectory exampleTrajectory =
            TrajectoryGenerator.generateTrajectory(

                new Pose2d(currentRobotPosition.getX(), currentRobotPosition.getY(), currentRobotPosition.getRotation()),

                interiorWaypoints,

                new Pose2d(desiredLocation.getX(), desiredLocation.getY(), desiredLocation.getRotation()),
                config);

        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        ETA = exampleTrajectory.getTotalTimeSeconds();
        SmartDashboard.putNumber("Auton Current Trajectory Estimated ETA", ETA);

        SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                exampleTrajectory,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);
        addCommands(
            new InstantCommand(() -> s_Swerve.setPose(exampleTrajectory.getInitialPose())),
            swerveControllerCommand
        );
    }
    public MoveToLocation(SwerveDrive s_Swerve, Pose2d desiredLocations){
        this(s_Swerve, desiredLocations, List.of());
    }

    public double ETA(){
        return ETA;
    }
}