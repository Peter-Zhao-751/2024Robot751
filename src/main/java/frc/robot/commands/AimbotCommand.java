package frc.robot.commands;

import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utility.StateMachine;

import java.util.Optional;

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
        FieldConstants.FieldElements[] fieldElements = FieldConstants.red;
        
        Optional<Alliance> alliance = DriverStation.getAlliance();

        if (alliance.isPresent() && alliance.get() == Alliance.Blue) fieldElements = FieldConstants.blue;
        
        FieldConstants.FieldElements element = fieldElements[1];

        double angle = Math.atan2(element.y - swerve.getPose().getY(), element.x - swerve.getPose().getX());
        double targetX = element.x + 3 * Math.cos(angle);
        double targetY = element.y + 3 * Math.sin(angle);
        
        angle = Math.toDegrees(angle);
        if (alliance.get() == Alliance.Blue) angle += 180;
        angle = (angle + 360) % 360;

        //FieldConstants.FieldElements closestElement = getFieldElements(fieldElements);

        StateMachine.setState(StateMachine.State.Aimbot);

        moveCommand = new MoveCommand(swerve, new Pose2d(targetX, targetY, new Rotation2d(angle)));
        moveCommand.initialize();
    }

    private FieldConstants.FieldElements getFieldElements(FieldConstants.FieldElements[] fieldElements) {
        double closestDistance = Double.MAX_VALUE;

        FieldConstants.FieldElements closestElement = null;

        Pose2d currentPose = swerve.getPose();
        for (FieldConstants.FieldElements fieldElement : fieldElements) {
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
        StateMachine.setState(StateMachine.State.Idle);
    }

    @Override
    public boolean isFinished() {
        return moveCommand == null || moveCommand.isFinished();
    }
}