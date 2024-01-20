package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
    private final NetworkTableEntry hasTarget;
    private final NetworkTable table;
    private final NetworkTableEntry target;
    private final NetworkTableEntry position; //TODO: implement this https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api  https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-coordinate-systems

    public Limelight() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        hasTarget = table.getEntry("tv");
        target = table.getEntry("tid");
        position = table.getEntry("botpose");

    }
    public boolean hasTarget() {
        return hasTarget.getDouble(0) > 0.9;
    }

    public double[] getValues() {
        if (hasTarget.getDouble(0) > 0.9) {
            return position.getDoubleArray(new double[6]);
        }
        return null;
    }

    public void debugDisplayValues() {
        double[] values = getValues();
        if (values != null) {
            SmartDashboard.putNumber("Limelight X", values[0]);
            SmartDashboard.putNumber("Limelight Y", values[1]);
            SmartDashboard.putNumber("Limelight Z", values[2]);
            SmartDashboard.putNumber("Limelight Pitch", values[3]);
            SmartDashboard.putNumber("Limelight Yaw", values[4]);
            SmartDashboard.putNumber("Limelight Roll", values[5]);
        }
    }

    public Pose2d getPose() {
        double[] values = getValues();
        if (values != null) {
            return new Pose2d(values[0], values[1], new Rotation2d(values[4]));
        }
        return null;
    }
}