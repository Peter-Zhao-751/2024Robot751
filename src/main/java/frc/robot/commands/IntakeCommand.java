package frc.robot.commands;

import frc.robot.Constants.Intake.IntakePositions;
import frc.robot.commands.TransferCommand.TransferMode;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.utility.StateMachine;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeCommand extends Command {

    public enum IntakeSwivelMode {
        Extend(IntakePositions.INTAKE, 20.0, TransferMode.Intake),
        Retract(IntakePositions.RETRACTED, 0.0, TransferMode.Outtake),
        Maintenance(IntakePositions.MAINTENANCE, 0.0, TransferMode.None),
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
        this.transferCommand = new TransferCommand(desiredSwivelState.speed, transferSubsystem, desiredSwivelState.transferMode, smartMode);
    }

    @Override
    public void initialize() {
        StateMachine.setState(StateMachine.State.Intake);
        intakeSubsystem.setSwivelPosition(desiredState.intakePosition);
        if (desiredState == IntakeSwivelMode.Extend) {
            transferCommand.schedule();
        }
    }

    @Override
    public void execute() {
        // The actual motor control is handled in the subsystem
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setSwivelPosition(IntakePositions.RETRACTED);
        // transferCommand.end(interrupted); i don't think this is necessary
        StateMachine.setState(StateMachine.State.Idle);
    }
    
    @Override
    public boolean isFinished() {
        return desiredState == IntakeSwivelMode.Retract || desiredState == IntakeSwivelMode.Maintenance || transferCommand.isFinished();
    }
}
