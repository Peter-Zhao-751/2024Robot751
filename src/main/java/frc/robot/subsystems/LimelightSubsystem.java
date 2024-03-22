package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.lib.math.Conversions;
import frc.robot.Constants;

import frc.robot.utility.TelemetrySubsystem;
import frc.robot.utility.TelemetryUpdater;
import frc.robot.utility.LimelightHelpers;

public class LimelightSubsystem {

	public enum LEDMode {
		PIPELINE(),
		OFF(),
		BLINK(),
		ON()
    }
		
    private static LimelightSubsystem instance;

    //TODO: implement this https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api
    // https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-coordinate-systems
    private final String name;

    public static LimelightSubsystem getInstance() {
        if (instance == null) instance = new LimelightSubsystem();
        return instance;
    }

	private LimelightSubsystem() {
        name = Constants.Limelight.name;
    }

    public boolean hasTarget() {
        return LimelightHelpers.getTV(this.name);
    }

    public double[] getValues() {
        return hasTarget() ? LimelightHelpers.getBotPose(this.name) : null;
    }

    public void setDriverMode() {
        LimelightHelpers.setCameraMode_Driver(this.name);
    }

    public void setVisionMode() {
        LimelightHelpers.setCameraMode_Processor(this.name);
    }

	public Pose2d getPose() {
		double[] values = getValues();
        return values != null ? new Pose2d(values[0], values[1], new Rotation2d(values[4])) : null;
    }

	public void setLEDMode(LEDMode mode) {
        switch (mode) {
            case PIPELINE -> LimelightHelpers.setLEDMode_PipelineControl(this.name);
            case OFF -> LimelightHelpers.setLEDMode_ForceOff(this.name);
            case BLINK -> LimelightHelpers.setLEDMode_ForceBlink(this.name);
            case ON -> LimelightHelpers.setLEDMode_ForceOn(this.name);
        }
    }

    public double getDistance() {
        // https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-estimating-distance#using-a-fixed-angle-camera
        double mountHeight = Constants.Limelight.height;
        double mountAngle = Constants.Limelight.angle; // might not be right

        double targetHeightRelative = LimelightHelpers.getTY(this.name);
        double targetHeightActual = LimelightHelpers.getBotPose(this.name)[2];

        double angle = Units.degreesToRadians(mountAngle + targetHeightRelative);

        return (targetHeightActual - mountHeight) / Math.tan(angle);
    }

    public void debugDisplayValues() {
        double[] values = getValues();
        if (values != null) {
            TelemetryUpdater.setTelemetryValue("LimelightSubsystem X Position", values[0]);
            TelemetryUpdater.setTelemetryValue("LimelightSubsystem Y Position", values[1]);
            TelemetryUpdater.setTelemetryValue("LimelightSubsystem Area", values[2]);
            TelemetryUpdater.setTelemetryValue("Estimated Distance", getDistance());
        }
    }
}