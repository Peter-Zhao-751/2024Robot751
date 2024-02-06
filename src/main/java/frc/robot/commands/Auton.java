package frc.robot.commands;

import java.util.ArrayList;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Command;

public class Auton extends SequentialCommandGroup {
    
    public Auton(SwerveDrive s_Swerve){
        //simple auton
        Move moveToLocation = new Move(s_Swerve, new Pose2d(3, 0, new Rotation2d(0)));
        
        Move moveToLocation2 = new Move(s_Swerve, new Pose2d(0, 0, new Rotation2d(0)));
        addCommands(moveToLocation, moveToLocation2);
        
    }
    public Auton(SwerveDrive s_Swerve, ArrayList<Command> path){
        // actual auton
        for (Command segment : path){
            addCommands(segment);
        }
    }
}