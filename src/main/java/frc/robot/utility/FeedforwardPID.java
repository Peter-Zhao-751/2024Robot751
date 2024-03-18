package frc.robot.utility;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.PIDCommand;

public class FeedforwardPID {
    private final PIDController pidController;
    private final SimpleMotorFeedforward feedforward;
    
    public FeedforwardPID(PIDController pidController, SimpleMotorFeedforward feedforward) {
        this.pidController = pidController;
        this.feedforward = feedforward;
    }
}
