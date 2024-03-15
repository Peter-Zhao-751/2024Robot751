package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utility.StateMachine;

import java.util.Optional;
import frc.robot.utility.AimAssistCalculations;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

public class AimbotCommand extends Command {
    private final SwerveSubsystem swerve;
    private MoveCommand moveCommand;

    public AimbotCommand(SwerveSubsystem s_Swerve) {
        this.swerve = s_Swerve;
        addRequirements(s_Swerve);
    }

    @Override
    public void initialize() {
        Constants.FieldConstants.FieldElements[] fieldElements = Constants.FieldConstants.red;
        
        Optional<Alliance> alliance = DriverStation.getAlliance();

        if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
            fieldElements = Constants.FieldConstants.blue;
        }

        Constants.FieldConstants.FieldElements closestElement = getFieldElements(fieldElements);

        StateMachine.setState(StateMachine.State.Aimbot);

        double targetAngle = AimAssistCalculations.calculateAngleToTarget(swerve.getPose(), 1);

        moveCommand = new MoveCommand(swerve, new Pose2d(closestElement.x, closestElement.y, new Rotation2d(targetAngle)));
        moveCommand.initialize();
    }

    private Constants.FieldConstants.FieldElements getFieldElements(Constants.FieldConstants.FieldElements[] fieldElements) {
        double closestDistance = Double.MAX_VALUE;

        Constants.FieldConstants.FieldElements closestElement = null;

        Pose2d currentPose = swerve.getPose();
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