package frc.robot.subsystems;

import java.io.File;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;


public class Limelight extends SubsystemBase {

    private enum LightMode {
        PIPELINE,
        OFF,
        BLINK,
        ON
    }

    private final NetworkTableEntry hasTarget;
    private final NetworkTable table;
    private final NetworkTableEntry target;
    private final NetworkTableEntry position; //TODO: implement this https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api  https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-coordinate-systems

    private final ShuffleboardTab tab;
    private final SendableChooser<LightMode> limeLightMode = new SendableChooser<>();
    private LightMode currentLightMode = LightMode.PIPELINE;

    public Limelight() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        hasTarget = table.getEntry("tv");
        target = table.getEntry("tid");
        position = table.getEntry("botpose");
        tab = Shuffleboard.getTab("Limelight");

        for (LightMode mode : LightMode.values()) {
            limeLightMode.addOption(mode.name(), mode);
        }
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
            tab.add("Limelight X", values[0]);
            tab.add("Limelight Y", values[1]);
            tab.add("Limelight Z", values[2]);
            tab.add("Limelight Pitch", values[3]);
            tab.add("Limelight Yaw", values[4]);
            tab.add("Limelight Roll", values[5]);
        }else{
            tab.add("Limelight X", "no target");
            tab.add("Limelight Y", "no target");
            tab.add("Limelight Z", "no target");
            tab.add("Limelight Pitch", "no target");
            tab.add("Limelight Yaw", "no target");
            tab.add("Limelight Roll", "no target");
        }
    }

    public Pose2d getPose() {
        double[] values = getValues();
        if (values != null) {
            return new Pose2d(values[0], values[1], new Rotation2d(values[4]));
        }
        return null;
    }

    public void setLightMode(LightMode mode) {
        table.getEntry("ledMode").setNumber(mode.ordinal());
    }

    @Override 
    public void periodic() {
        debugDisplayValues();
        LightMode newMode = limeLightMode.getSelected();
        if (newMode != currentLightMode) {
            setLightMode(newMode);
            currentLightMode = newMode;
        }
    }
}