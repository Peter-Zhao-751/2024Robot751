package frc.robot.commands.lowLevelCommands;

import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TransferSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Intake extends Command {
    private IntakeSubsystem intakeSubsystem;
    private TransferSubsystem transferSubsystem;

    private double intakeDuration;


    private double startTime;

    public Intake(IntakeSubsystem intakeSubsystem, TransferSubsystem transferSubsystem, boolean extend) {
        this.intakeSubsystem = intakeSubsystem;
        this.transferSubsystem = transferSubsystem;
        intakeDuration = -1;
        addRequirements(intakeSubsystem);
    }
    public Intake(IntakeSubsystem intakeSubsystem, TransferSubsystem transferSubsystem, double intakeDuration) {
        this(intakeSubsystem, transferSubsystem, false);
        this.intakeDuration = intakeDuration;
    }
    public Intake(IntakeSubsystem intakeSubsystem, TransferSubsystem transferSubsystem, double intakeDuration, boolean extend) {
        this(intakeSubsystem, transferSubsystem, extend);
        this.intakeDuration = intakeDuration;
    }
    public Intake(IntakeSubsystem intakeSubsystem, TransferSubsystem transferSubsystem) {
        this(intakeSubsystem, transferSubsystem, false);
    }
    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();
        intakeSubsystem.setSwivelPosition(Constants.Intake.kSwivelExtendedAngle);
    }

    @Override
    public void execute() {
        // The actual motor control is handled in the subsystem
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setSwivelPosition(Constants.Intake.kSwivelRetractedAngle);
        transferSubsystem.setIntakeTransfer(0);
    }
    @Override
    public boolean isFinished() {
        return intakeDuration > 0 && System.currentTimeMillis() - startTime >= intakeDuration;
    }

}
