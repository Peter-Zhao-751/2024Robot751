package frc.robot.commands.movementCommands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants;

public class SwerveRotateControllerCommand extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final ProfiledPIDController thetaController;
    private final Rotation2d targetAngle;

    public SwerveRotateControllerCommand(Rotation2d targetAngle) {
        this.swerveSubsystem = SwerveSubsystem.getInstance();
        this.targetAngle = targetAngle;
		addRequirements(swerveSubsystem);
		
        thetaController = new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0,
                new TrapezoidProfile.Constraints(
                        Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecond,
                        Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared));
		thetaController.enableContinuousInput(-Math.PI, Math.PI);
		thetaController.setTolerance(3);
    }

    @Override
    public void initialize() {
        // Ensure controller is starting fresh
        thetaController.reset(swerveSubsystem.getPose().getRotation().getRadians());
    }

    @Override
    public void execute() {
        double rotationSpeed = thetaController.calculate(
                swerveSubsystem.getPose().getRotation().getRadians(),
                targetAngle.getRadians());

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, rotationSpeed);
        SwerveModuleState[] moduleStates = swerveSubsystem.getKinematics().toSwerveModuleStates(chassisSpeeds);

        swerveSubsystem.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted) {
        // Optionally, stop the swerve modules or set them to a neutral state
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        // Command is finished when the robot's rotation is within a small threshold of the target
        return thetaController.atGoal();
    }
}