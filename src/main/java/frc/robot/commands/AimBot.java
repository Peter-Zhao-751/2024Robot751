package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class AimBot extends SequentialCommandGroup {
    public AimBot(SwerveDrive s_Swerve){
        //TODO: compare how far is everything, and choses the closest one to do
        double closestDistance = Double.MAX_VALUE;
        Constants.FieldConstants.FieldElements closestIndex = Constants.FieldConstants.all[0];
        
        for(int i = 0; i < Constants.FieldConstants.all.length; i++){
           
        }

        addCommands(new Move(s_Swerve, new Pose2d(3, 0, new Rotation2d(0))));

    }

    private void determineAim(){

    }
}