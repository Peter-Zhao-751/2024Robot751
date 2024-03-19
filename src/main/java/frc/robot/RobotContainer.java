package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.AutonCommand;
import frc.robot.commands.CANdleController;
import frc.robot.subsystems.*;
import frc.robot.utility.Barn2PathInterpreter;
import frc.robot.utility.ControlBoard;

import java.io.File;

public class RobotContainer {
    /* Subsystems */
    //private final CANdle s_CANdle = new CANdle();
    private final ShooterSubsystem s_Shooter = ShooterSubsystem.getInstance();
    private final IntakeSubsystem s_Intake = IntakeSubsystem.getInstance();
    //private final ClimberSubsystem s_Climber = ClimberSubsystem.getInstance();
    private final SwerveSubsystem s_Swerve = SwerveSubsystem.getInstance();
    private final TransferSubsystem s_Transfer = TransferSubsystem.getInstance();
    private final PowerSubsystem s_PDH = PowerSubsystem.getInstance();

    private final Barn2PathInterpreter u_Barn2PathInterpreter = new Barn2PathInterpreter();

    /* The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        //CANdleController.setCandle(CANdleSubsystem.getInstance());
        ControlBoard.getInstance();
    }

    public Command getAutonomousCommand() {
        return new AutonCommand();
    }

    public Command getAutonomousCommand(File path) {
        if (path != null) {
            try {
                return new AutonCommand(u_Barn2PathInterpreter.getAutonCommands(path));
            } catch (Exception e) {
                System.err.println("Error: " + e);
            }
        }
        System.err.println("No path file found");
        return new AutonCommand();
    }
}