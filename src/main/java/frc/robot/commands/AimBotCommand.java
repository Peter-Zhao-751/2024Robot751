package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utility.StateMachine;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

public class AimBotCommand extends Command {
    private final SwerveSubsystem s_Swerve;
    private MoveCommand moveCommand;

    public AimBotCommand(SwerveSubsystem s_Swerve) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
    }

    @Override
    public void initialize() {
        // checking the color of the robot
        Constants.FieldConstants.FieldElements[] fieldElements = Constants.FieldConstants.red;
        
        Optional<Alliance> alliance = DriverStation.getAlliance();

        if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
            fieldElements = Constants.FieldConstants.blue;
        }

        double closestDistance = Double.MAX_VALUE;

        Constants.FieldConstants.FieldElements closestElement = null;

        Pose2d currentPose = s_Swerve.getPose();
        for (int i = 0; i < fieldElements.length; i++) {
            double distance = Math.sqrt(Math.pow(currentPose.getX() - fieldElements[i].x, 2) + Math.pow(currentPose.getY() - fieldElements[i].y, 2));
            if (distance < closestDistance ) {
                closestElement = fieldElements[i];
                closestDistance = distance;
            }
        }

        StateMachine.setState(StateMachine.State.Aimbot);

        moveCommand = new MoveCommand(s_Swerve, new Pose2d(closestElement.x, closestElement.y, new Rotation2d(0)));
        moveCommand.initialize();
    }

    @Override
    public void execute() {
        if (moveCommand != null) {
            moveCommand.execute();
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (moveCommand != null) moveCommand.end(interrupted);
        
        StateMachine.setState(StateMachine.State.Idle);
    }

    @Override
    public boolean isFinished() {
        return moveCommand != null && moveCommand.isFinished();
    }
}