package frc.robot.commands.gamepieceCommands;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.utility.ControlBoard;
import frc.robot.utility.StateMachine;
import frc.robot.utility.TelemetryUpdater;
import edu.wpi.first.wpilibj2.command.Command;

public class ShootCommand extends Command{
    private final ShooterSubsystem shooterSubsystem;
    private final TransferSubsystem transferSubsystem;
    private final IntakeSubsystem intakeSubsystem;

    private final LimelightSubsystem limelightSubsystem;
    private final ControlBoard controlBoard;

    private final boolean smartMode;
    private boolean started;

    private ControlBoard.Mode mode;

    public ShootCommand(boolean smartMode) {
        this.shooterSubsystem = ShooterSubsystem.getInstance();
        this.transferSubsystem = TransferSubsystem.getInstance();
        this.intakeSubsystem = IntakeSubsystem.getInstance();

        this.limelightSubsystem = LimelightSubsystem.getInstance();
        this.controlBoard = ControlBoard.getInstance();

        this.smartMode = smartMode;

        addRequirements(shooterSubsystem, transferSubsystem, intakeSubsystem);
    }

    public ShootCommand() {
        this(true);
    }

    @Override
    public void initialize() {
        StateMachine.setState(StateMachine.State.Shoot);
        mode = ControlBoard.getInstance().getMode();
        started = !smartMode;

        if (mode == ControlBoard.Mode.Speaker) {
            double power = Constants.Shooter.maxShooterSpeed;
            if(limelightSubsystem.hasTarget()) {
                double dist = limelightSubsystem.getDistance();
                power = calculatePower(dist);
            }

            shooterSubsystem.setSpeed(power + controlBoard.shooterSpeed());
            if (!smartMode) transferSubsystem.setTransferSpeed(Constants.Transfer.intakeTransferSpeed);
        } else {
            intakeSubsystem.setSwivelPosition(Constants.Intake.kAmpAngle);
            if (!smartMode) intakeSubsystem.setIntakeSpeed(-Constants.Shooter.intakeAmpSpeed);
        }
    }

    private double calculatePower(double dist) {
        return 0;
    }

    @Override
    public void execute() {
        TelemetryUpdater.setTelemetryValue("Shooter/At shooter Speed", shooterSubsystem.isAtTargetSpeed());
        if (!started && smartMode) {
            if (mode == ControlBoard.Mode.Speaker && shooterSubsystem.isAtTargetSpeed() && shooterSubsystem.getShooterSpeed() > 5) {
                transferSubsystem.setTransferSpeed(Constants.Transfer.intakeTransferSpeed);
                started = true;
            } else if (mode == ControlBoard.Mode.Amp && intakeSubsystem.closeToSetpoint() && intakeSubsystem.getSwivelPosition() < Constants.Intake.kRetractedAngle - 3) { // TODO: remove angle check if working
                intakeSubsystem.setIntakeSpeed(-Constants.Shooter.intakeAmpSpeed);
                started = true;
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.setSpeed(0);
        transferSubsystem.setTransferSpeed(0);
        intakeSubsystem.setIntakeSpeed(0);
        intakeSubsystem.setSwivelPosition(Constants.Intake.kRetractedAngle);
        StateMachine.setState(StateMachine.State.Idle);
    }
}
