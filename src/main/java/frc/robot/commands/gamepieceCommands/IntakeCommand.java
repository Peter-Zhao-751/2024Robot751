package frc.robot.commands.gamepieceCommands;

import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.utility.StateMachine;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeCommand extends Command {
    private final IntakeSubsystem intakeSubsystem;
    private final TransferSubsystem transferSubsystem;
	private final boolean autonMode;
	private double intakeStartTime;
	private boolean hasStartedIntake;

	public IntakeCommand(boolean autonMode) {
		this.intakeSubsystem = IntakeSubsystem.getInstance();
		this.transferSubsystem = TransferSubsystem.getInstance();
		this.autonMode = autonMode;
	}

	public IntakeCommand() {
		this(false);
	}

    @Override
    public void initialize() {
        StateMachine.setState(StateMachine.State.Intake);
		intakeSubsystem.setSwivelPosition(Constants.Intake.kIntakeAngle);
        transferSubsystem.setIntakeTransfer(Constants.Transfer.intakeTransferSpeed);
		transferSubsystem.setShooterTransfer(-50);
		intakeStartTime = 0;
		hasStartedIntake = false;
    }

    @Override
    public void execute() {
		if (intakeSubsystem.greaterThanSetpoint()) {
			hasStartedIntake = true;
			intakeStartTime = System.currentTimeMillis();
			intakeSubsystem.setIntakeSpeed(Constants.Intake.intakeSpeed);
            //intakeSubsystem.setIntakeSpeed(currentMode == Mode.Speaker ? Constants.Intake.speakerIntakeSpeed : Constants.Intake.ampIntakeSpeed);
        }
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setIntakeSpeed(0);
        intakeSubsystem.setSwivelPosition(Constants.Intake.kRetractedAngle);
        transferSubsystem.stop();
        StateMachine.setState(StateMachine.State.Idle);
    }

    @Override
	public boolean isFinished() {
		boolean smartBeamBreak = transferSubsystem.beamBroken() && hasStartedIntake && System.currentTimeMillis() - intakeStartTime > Constants.Intake.minIntakeTime * 1000;
		return smartBeamBreak || (autonMode && hasStartedIntake && System.currentTimeMillis() - intakeStartTime > Constants.Intake.intakeTime);
        // return switch (currentMode) {
        //     case Speaker -> transferSubsystem.beamBroken();
        //     case Amp -> intakeSubsystem.beamBroken();
        //     default -> false;
        // };
    }
}
