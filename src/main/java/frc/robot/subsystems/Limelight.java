package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.*;
import frc.robot.utility.TelemetryUpdater;


public class Limelight {
    private static Limelight instance;

    private final NetworkTableEntry hasTarget;
    private final NetworkTable table;
    private final NetworkTableEntry target;
    private final NetworkTableEntry position; //TODO: implement this https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api  https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-coordinate-systems

    public static Limelight getInstance() {
        if (instance == null) instance = new Limelight();
        return instance;
    }

    private Limelight() {
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

    public Pose2d getPose() {
        double[] values = getValues();
        if (values != null) {
            return new Pose2d(values[0], values[1], new Rotation2d(values[4]));
        }
        return null;
    }

    public void debugDisplayValues() {
        double[] values = getValues();
        if (values != null) {
            TelemetryUpdater.setTelemetryValue("Limelight X Position", values[0]);
            TelemetryUpdater.setTelemetryValue("Limelight Y Position", values[1]);
        }
    }
}