package frc.robot.commands.gamepieceCommands;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.utility.ControlBoard;
import frc.robot.utility.StateMachine;
import frc.robot.utility.TelemetryUpdater;
import edu.wpi.first.wpilibj2.command.Command;

public class ShootCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private final TransferSubsystem transferSubsystem;
    private final IntakeSubsystem intakeSubsystem;

    private enum AmpState {
        MovingIntakeToLoad,
        TransferToIntake,
        MovingIntakeToShoot,
        Shoot;
    }

    private final LimelightSubsystem limelightSubsystem;

    private final InterpolatingDoubleTreeMap shooterSpeedMap;

    private final boolean autonMode;
    private boolean startedShooting;
    private double shootingStartTime;

    private AmpState ampShootingState;

    private ControlBoard.Mode mode;

    public ShootCommand(boolean autonMode) {
        this.shooterSubsystem = ShooterSubsystem.getInstance();
        this.transferSubsystem = TransferSubsystem.getInstance();
        this.intakeSubsystem = IntakeSubsystem.getInstance();

        this.limelightSubsystem = LimelightSubsystem.getInstance();

        this.shooterSpeedMap = constructShooterSpeedMap();

        this.autonMode = autonMode;
        ampShootingState = null;

        addRequirements(shooterSubsystem, transferSubsystem, intakeSubsystem);
    }

    public ShootCommand() {
        this(false);
    }

    public InterpolatingDoubleTreeMap constructShooterSpeedMap() {
        InterpolatingDoubleTreeMap map = new InterpolatingDoubleTreeMap();
        // TODO: add points
        // Dist, Speed
        map.put(Constants.Shooter.targetDistance, Constants.Shooter.maxShooterSpeed);
        return map;
    }

    @Override
    public void initialize() {
        StateMachine.setState(StateMachine.State.Shoot);
        mode = ControlBoard.getInstance().getMode();
        startedShooting = false;
        shootingStartTime = 0;

        if (mode == ControlBoard.Mode.Speaker) {
            double power = Constants.Shooter.maxShooterSpeed;
            if (limelightSubsystem.hasTarget()) {
                double dist = limelightSubsystem.getDistance();
                power = shooterSpeedMap.get(dist);
            }
            shooterSubsystem.setSpeed(power + ControlBoard.getInstance().shooterSpeedOffset());
        } else {
            if (!intakeSubsystem.beamBroken()) {
                intakeSubsystem.setSwivelPosition(Constants.Intake.kIntakeAngle);
                ampShootingState = AmpState.MovingIntakeToLoad;
            } else {
                intakeSubsystem.setSwivelPosition(Constants.Intake.kAmpAngle);
                ampShootingState = AmpState.MovingIntakeToShoot;
            }
        }
    }

    @Override
    public void execute() {
        TelemetryUpdater.setTelemetryValue("Shooter/At shooter Speed", shooterSubsystem.isAtTargetSpeed());
        if (!startedShooting) {
            if (mode == ControlBoard.Mode.Speaker && shooterSubsystem.isAtTargetSpeed()) {
                transferSubsystem.setTransferSpeed(Constants.Transfer.intakeTransferSpeed);
                startedShooting = true;
                shootingStartTime = System.currentTimeMillis();
            } else if (mode == ControlBoard.Mode.Amp) {

                TelemetryUpdater.setTelemetryValue("Shooter/Amp State", ampShootingState.name());

                switch (ampShootingState) {
                    case MovingIntakeToLoad:
                        if (intakeSubsystem.greaterThanSetpoint()) {
                            intakeSubsystem.setIntakeSpeed(-Constants.Shooter.scoreAmpSpeed/1.5);
                            transferSubsystem.setTransferSpeed(-Constants.Shooter.scoreAmpSpeed/1.5);
                            ampShootingState = AmpState.TransferToIntake;
                        }
                        break;
                    case TransferToIntake:
                        if (intakeSubsystem.beamBroken()) {
                            intakeSubsystem.setIntakeSpeed(0);
                            transferSubsystem.setTransferSpeed(0);
                            intakeSubsystem.setSwivelPosition(Constants.Intake.kAmpAngle);
                            ampShootingState = AmpState.MovingIntakeToShoot;
                        }
                        break;
                    case MovingIntakeToShoot:
                        if (intakeSubsystem.closeToSetpoint()) {
                            intakeSubsystem.setIntakeSpeed(-Constants.Shooter.scoreAmpSpeed);
                            startedShooting = true;
                            shootingStartTime = System.currentTimeMillis();
                            ampShootingState = AmpState.Shoot;
                        }
                        break;
                    default:
                        System.err.println("should not be here, amp shoot is being weird");
                        break;
                }
            }
        }
    }

    @Override
    public boolean isFinished() {
        return autonMode && startedShooting && System.currentTimeMillis() - shootingStartTime > Constants.Shooter.feedTime * 1000;
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
