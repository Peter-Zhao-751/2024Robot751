package frc.robot.commands;

import java.util.ArrayList;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Auton extends SequentialCommandGroup {
    MoveToLocation moveToLocation;
    public Auton(SwerveDrive s_Swerve){
        MoveToLocation moveToLocation = new MoveToLocation(s_Swerve, new Pose2d(3, 0, new Rotation2d(0)));
        
        MoveToLocation moveToLocation2 = new MoveToLocation(s_Swerve, new Pose2d(0, 0, new Rotation2d(0)));
        addCommands(moveToLocation, moveToLocation2);

        
    }
    public Auton(SwerveDrive s_Swerve, ArrayList<AutonCommandSegment> path){
        
    }
}