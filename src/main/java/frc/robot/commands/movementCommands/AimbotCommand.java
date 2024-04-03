package frc.robot.commands.movementCommands;

import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utility.StateMachine;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

public class AimbotCommand extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private MoveCommand moveCommand;
    private final LimelightSubsystem limelightSubsystem;

    public AimbotCommand() {
        this.swerveSubsystem = SwerveSubsystem.getInstance();
        this.limelightSubsystem = LimelightSubsystem.getInstance();

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        limelightSubsystem.setVisionMode();
        limelightSubsystem.setLEDMode(LimelightSubsystem.LEDMode.ON);

        FieldConstants.FieldElements[] fieldElements = FieldConstants.red;

        Optional<Alliance> alliance = DriverStation.getAlliance();

        if (alliance.isPresent() && alliance.get() == Alliance.Blue) fieldElements = FieldConstants.blue;

        FieldConstants.FieldElements element = fieldElements[1];

        double angle = Math.atan2(element.y - swerveSubsystem.getPose().getY(), element.x - swerveSubsystem.getPose().getX());

        double targetX = element.x - 300 * Math.cos(angle);
        double targetY = element.y - 300 * Math.sin(angle);

		angle = Math.toDegrees(angle);
		angle = (angle + 360) % 360;

        //FieldConstants.FieldElements closestElement = getFieldElements(fieldElements);

        StateMachine.setState(StateMachine.State.Aimbot);

        moveCommand = new MoveCommand(new Pose2d(targetX, targetY, new Rotation2d(angle)));
        moveCommand.initialize();
    }

    private FieldConstants.FieldElements getFieldElements(FieldConstants.FieldElements[] fieldElements) {
        double closestDistance = Double.MAX_VALUE;

        FieldConstants.FieldElements closestElement = null;

        Pose2d currentPose = swerveSubsystem.getPose();
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
        if (moveCommand != null) moveCommand.execute();
    }

    @Override
    public void end(boolean interrupted) {
        StateMachine.setState(StateMachine.State.Idle);
        limelightSubsystem.setLEDMode(LimelightSubsystem.LEDMode.OFF);
        if (moveCommand != null) moveCommand.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return moveCommand == null || moveCommand.isFinished();
    }
}