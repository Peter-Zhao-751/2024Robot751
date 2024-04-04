package frc.robot.commands.movementCommands;

import java.util.ArrayList;
import java.util.Optional;

import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Command;

public class AutonCommand extends SequentialCommandGroup {

	public AutonCommand() {
		Optional<Alliance> alliance = DriverStation.getAlliance();
        //simple auton
        MoveCommand moveToLocation = new MoveCommand(new Pose2d(7, 0, new Rotation2d(0)));

        //MoveCommand moveToLocation2 = new MoveCommand(s_Swerve, new Pose2d(1, 0, new Rotation2d(0)));
        addCommands(moveToLocation);
    }

    public AutonCommand(ArrayList<Command> path){
        // actual auton
        for (Command segment : path){
            addCommands(segment);
        }
    }
}