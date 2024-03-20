package frc.robot.utility;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.RobotController;

public class FeedforwardPID {
    private final PIDController pidController;
    private final SimpleMotorFeedforward feedforward;

    private double feedforwardOutput;
    private double pidOutput;

    public FeedforwardPID(double kS, double kV, double kA, double kP, double kI, double kD) {
        this.pidController = new PIDController(kP, kI, kD);
        this.feedforward = new SimpleMotorFeedforward(kS, kV, kA);
        feedforwardOutput = 0;
        pidOutput = 0;
    }

    public double calculate(double measurement, double setpoint) {
        double maxVoltage = RobotController.getBatteryVoltage() * 0.95;

        feedforwardOutput = feedforward.calculate(setpoint);
        pidOutput = pidController.calculate(measurement, setpoint);

        double totalOutput = Math.min(feedforwardOutput + pidOutput, maxVoltage);
        totalOutput = Math.max(totalOutput, -maxVoltage);
        return totalOutput;
    }

    public void debugPrintouts(String name) {
        TelemetryUpdater.setTelemetryValue(name + " Feedforward Output", feedforwardOutput);
        TelemetryUpdater.setTelemetryValue(name + " PID Output", pidOutput);
        TelemetryUpdater.setTelemetryValue(name + "total Output", feedforwardOutput + pidOutput);
    }
}
