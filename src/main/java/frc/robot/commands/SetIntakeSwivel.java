package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class SetIntakeSwivel extends Command{
    private IntakeSubsystem intakeSubsystem;
    private PIDController swivelPIDController;
    private double targetAngle;

    public SetIntakeSwivel(IntakeSubsystem intakeSubsystem, boolean extend){
        this.intakeSubsystem = intakeSubsystem;
        swivelPIDController = new PIDController(Constants.Intake.kPSwivelController, Constants.Intake.kISwivelController, Constants.Intake.kDSwivelController);
        targetAngle = extend ? Constants.Intake.kSwivelExtendedAngle : Constants.Intake.kSwivelRetractedAngle;
        addRequirements(intakeSubsystem);
    }
    public SetIntakeSwivel(IntakeSubsystem intakeSubsystem){
        this(intakeSubsystem, false);
    }
    @Override
    public void initialize() {
        // could play a sound or something
    }
    @Override
    public void execute() {
        double currentPosition = intakeSubsystem.getSwivelPosition();
        double speed = Constants.Intake.maxSwivelSpeed * swivelPIDController.calculate(currentPosition, targetAngle);
        intakeSubsystem.setSwivelSpeed(speed);
    }
    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopAll();
    }
    @Override
    public boolean isFinished() {
        return Math.abs(targetAngle - intakeSubsystem.getSwivelPosition()) < 0.1;
    }
}
