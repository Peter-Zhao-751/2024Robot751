package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class AimBot extends Command {
    private final SwerveDrive s_Swerve;
    private Move move;

    public AimBot(SwerveDrive s_Swerve) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
    }

    @Override
    public void initialize() {
        
        Pose2d currentPose = s_Swerve.getPose();
        Constants.FieldConstants.FieldElements[] fieldElements = Constants.FieldConstants.all;
        double closestDistance = Double.MAX_VALUE;
        Constants.FieldConstants.FieldElements closestElement = null;

        for (int i = 0; i < fieldElements.length; i++) {
            double distance = Math.sqrt(Math.pow(currentPose.getX() - fieldElements[i].x, 2) + Math.pow(currentPose.getY() - fieldElements[i].y, 2));
            if (distance < closestDistance ) {
                closestElement = fieldElements[i];
                closestDistance = distance;
            }
        }
        //TODO: Add logic to check if it is within range
        //TODO: Add logic to aim at the closest element 
        move = new Move(s_Swerve, new Pose2d(closestElement.x, closestElement.y, new Rotation2d(0)));
        move.initialize();
    }

    @Override
    public void execute() {
        if (move != null) {
            move.execute();
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (move != null) {
            move.end(interrupted);
        }
    }

    @Override
    public boolean isFinished() {
        return move != null && move.isFinished();
    }
}