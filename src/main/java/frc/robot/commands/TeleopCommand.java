package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utility.StateMachine;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;



public class TeleopCommand extends Command {
    private final SwerveSubsystem s_Swerve;
    private final DoubleSupplier translationSup;
    private final DoubleSupplier strafeSup;
    private final DoubleSupplier rotationSup;
    private final BooleanSupplier robotCentricSup;
    private final BooleanSupplier preciseControl;

    public TeleopCommand(DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, BooleanSupplier preciseControl) {
        this.s_Swerve = SwerveSubsystem.getInstance();

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.preciseControl = preciseControl;

        addRequirements(s_Swerve);
    }
    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/        
        double translationVal = translationSup.getAsDouble();
        double strafeVal = strafeSup.getAsDouble();
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

        if (Math.hypot(translationVal, strafeVal) < Constants.stickDeadband * Math.sqrt(2)) {
            translationVal = strafeVal = 0;
        }

        /* Drive */
        boolean isDriving = s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            !robotCentricSup.getAsBoolean(), 
            true, 
            preciseControl.getAsBoolean()
        );
        if (!StateMachine.isPerformingAction()){
            if (isDriving) StateMachine.setState(StateMachine.State.TeleopDrive);
            else StateMachine.setState(StateMachine.State.Idle);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        System.err.println("TeleopCommand was interrupted");
    }
}