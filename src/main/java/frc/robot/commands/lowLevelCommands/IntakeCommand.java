package frc.robot.commands.lowLevelCommands;

import frc.robot.Constants;
import frc.robot.commands.lowLevelCommands.TransferCommand.TransferMode;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.utility.StateMachine;
import frc.robot.utility.TelemetryUpdater;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeCommand extends Command {

    public enum IntakeSwivelMode {
        Extend(Constants.Intake.kIntakeAngle, 600.0, TransferMode.Intake),
        Retract(Constants.Intake.kRetractedAngle, 0.0, null),
        Maintenance(Constants.Intake.kMaintenanceAngle, 0.0, null),
        Amp(Constants.Intake.kAmpAngle, 600.0, TransferMode.Outtake);

        public final double intakePosition;
        public final double speed;
        public final TransferMode transferMode;
        
        IntakeSwivelMode(double intakePosition, double intakeSpeed, TransferMode transferMode) {
            this.intakePosition = intakePosition;
            this.speed = intakeSpeed;
            this.transferMode = transferMode;
        }
    }

    private final IntakeSubsystem intakeSubsystem;
    private final TransferCommand transferCommand;
    private final IntakeSwivelMode desiredState;

    public IntakeCommand(IntakeSubsystem intakeSubsystem, TransferSubsystem transferSubsystem, IntakeSwivelMode desiredSwivelState, boolean smartMode) {
        this.desiredState = desiredSwivelState;
        this.intakeSubsystem = intakeSubsystem;
        if (desiredSwivelState.transferMode != null) this.transferCommand = new TransferCommand(desiredSwivelState.speed * 1.5, transferSubsystem, desiredSwivelState.transferMode, smartMode);
        else this.transferCommand = null;
    }

    @Override
    public void initialize() {
        StateMachine.setState(StateMachine.State.Intake);
        intakeSubsystem.setSwivelPosition(desiredState.intakePosition);
        if (transferCommand != null) {
            transferCommand.initialize();
            intakeSubsystem.setIntakeSpeed(desiredState.speed);
        }
    }

    @Override
    public void execute() {
        // The actual motor control is handled in the subsystem
    }

    @Override
    public void end(boolean interrupted) {
        if (!desiredState.equals(IntakeSwivelMode.Maintenance)) intakeSubsystem.setSwivelPosition(IntakeSwivelMode.Retract.intakePosition);
        if (transferCommand != null) transferCommand.end(interrupted);
        intakeSubsystem.stopAll();
        StateMachine.setState(StateMachine.State.Idle);
    }
    
    @Override
    public boolean isFinished() {
        return transferCommand == null || transferCommand.isFinished();
    }
}
