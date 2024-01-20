package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;


import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Intake extends Command {
    private IntakeSubsystem intakeSubsystem;
    private double startTime;
    public Intake(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }
    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();
        //motorSubsystem.runMotor(speed);
    }

    @Override
    public void execute() {
        // The actual motor control is handled in the subsystem
    }

    @Override
    public void end(boolean interrupted) {
        //motorSubsystem.stopMotor(); // Stop the motor when the command ends
    }
    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() - startTime >= 3000; // Ends the command after 3 seconds
    }

}
