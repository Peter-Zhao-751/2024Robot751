package frc.robot.commands.lowLevelCommands;

import frc.robot.Constants.Intake.IntakePositions;
import frc.robot.commands.lowLevelCommands.TransferCommand.TransferMode;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.utility.StateMachine;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeCommand extends Command {

    public enum IntakeSwivelMode {
        Extend(IntakePositions.INTAKE, 20.0, TransferMode.Intake),
        Retract(IntakePositions.RETRACTED, 0.0, null),
        Maintenance(IntakePositions.MAINTENANCE, 0.0, null),
        Amp(IntakePositions.AMP, 20.0, TransferMode.Outtake);

        public final IntakePositions intakePosition;
        public final double speed;
        public final TransferMode transferMode;
        
        IntakeSwivelMode(IntakePositions intakePosition, double intakeSpeed, TransferMode transferMode) {
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
        if (desiredSwivelState.transferMode != null) this.transferCommand = new TransferCommand(desiredSwivelState.speed, transferSubsystem, desiredSwivelState.transferMode, smartMode);
        else this.transferCommand = null;
    }

    @Override
    public void initialize() {
        StateMachine.setState(StateMachine.State.Intake);
        intakeSubsystem.setSwivelPosition(desiredState.intakePosition);
        intakeSubsystem.setIntakeSpeed(40);
        if (transferCommand != null) transferCommand.schedule();
    }

    @Override
    public void execute() {
        // The actual motor control is handled in the subsystem
    }

    @Override
    public void end(boolean interrupted) {
        if (desiredState.intakePosition != IntakePositions.MAINTENANCE) intakeSubsystem.setSwivelPosition(IntakePositions.RETRACTED);
        if (transferCommand != null) transferCommand.end(interrupted);
        intakeSubsystem.stopAll();
        StateMachine.setState(StateMachine.State.Idle);
    }
    
    @Override
    public boolean isFinished() {
        return transferCommand == null || transferCommand.isFinished();
    }
}
