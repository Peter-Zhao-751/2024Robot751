package frc.robot.utility;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;

public class AimAssistCalculations {
    private AimAssistCalculations(){
        throw new UnsupportedOperationException("This is a utility class and should not be instantiated.");
    }

    public static double calculateAngleToTarget(Pose2d currentRobotPose, int fieldElementIndex) { // 0 is amp, 1 is speaker
        FieldConstants.FieldElements[] fieldElements = FieldConstants.red;
        
        Optional<Alliance> alliance = DriverStation.getAlliance();

        if (alliance.isPresent() && alliance.get() == Alliance.Blue) fieldElements = FieldConstants.blue;

        FieldConstants.FieldElements speaker = fieldElements[fieldElementIndex];
        
        StateMachine.setState(StateMachine.State.Aimbot);

        // Calculate the angle to turn towards the speaker
        double targetAngle = Math.toDegrees(Math.atan2(speaker.y - currentRobotPose.getY(), speaker.x - currentRobotPose.getX()));
        if (alliance.get() == Alliance.Blue) targetAngle += 180;
        
        return (targetAngle + 360) % 360;
    }
}
