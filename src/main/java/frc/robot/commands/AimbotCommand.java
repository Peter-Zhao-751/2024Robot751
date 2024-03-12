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

public class AimbotCommand extends Command {
    private final SwerveSubsystem s_Swerve;
    private MoveCommand moveCommand;

    public AimbotCommand(SwerveSubsystem s_Swerve) {
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

        Constants.FieldConstants.FieldElements closestElement = getFieldElements(fieldElements);

        StateMachine.setState(StateMachine.State.Aimbot);

        moveCommand = new MoveCommand(s_Swerve, new Pose2d(closestElement.x, closestElement.y, new Rotation2d(0)));
        moveCommand.initialize();
    }

    private Constants.FieldConstants.FieldElements getFieldElements(Constants.FieldConstants.FieldElements[] fieldElements) {
        double closestDistance = Double.MAX_VALUE;

        Constants.FieldConstants.FieldElements closestElement = null;

        Pose2d currentPose = s_Swerve.getPose();
        for (Constants.FieldConstants.FieldElements fieldElement : fieldElements) {
            double distance = Math.sqrt(Math.pow(currentPose.getX() - fieldElement.x, 2) + Math.pow(currentPose.getY() - fieldElement.y, 2));
            if (distance < closestDistance) {
                closestElement = fieldElement;
                closestDistance = distance;
            }
        }
        return closestElement;
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