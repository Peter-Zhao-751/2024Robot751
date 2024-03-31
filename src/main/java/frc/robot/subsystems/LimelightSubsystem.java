package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import frc.lib.util.LimelightHelpers;
import frc.robot.utility.TelemetryUpdater;

public class LimelightSubsystem extends SubsystemBase {
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

    public boolean forceLedOff = false;

    public static LimelightSubsystem getInstance() {
        if (instance == null) instance = new LimelightSubsystem();
        return instance;
    }

	private LimelightSubsystem() {
        name = Constants.Limelight.name;
    }

    public void toggleLeds() {
        forceLedOff = !forceLedOff;
    }

    public boolean hasTarget() {
        return LimelightHelpers.getTV(this.name);
    }

    public double[] getValues() {
        return hasTarget() ? LimelightHelpers.getBotPose_wpiBlue(this.name) : null;
    }

    public double getYaw() {
        return LimelightHelpers.getBotPose2d(this.name).getRotation().getDegrees();
    }

    public void setDriverMode() {
        LimelightHelpers.setCameraMode_Driver(this.name);
    }

    public void setVisionMode() {
        LimelightHelpers.setCameraMode_Processor(this.name);
    }

    /**
     * 0 is x, 1 is y, 2 z, 3 rotation x, 4 rotation y, 5 rotation z
     * @return Pose2d, null if no target
     */
	public Pose2d getPose() {
		double[] values = getValues();
        return values != null ? new Pose2d(values[0], values[1], new Rotation2d(values[4])) : null;
    }

	public void setLEDMode(LEDMode mode) {
        if (forceLedOff) mode = LEDMode.OFF;
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

    @Override
    public void periodic() {
        TelemetryUpdater.setTelemetryValue("LimelightSubsystem Has Target", hasTarget());
        double[] values = getValues();
        if (values != null) {
            TelemetryUpdater.setTelemetryValue("LimelightSubsystem X Position", values[0]);
			TelemetryUpdater.setTelemetryValue("LimelightSubsystem Y Position", values[1]);
			TelemetryUpdater.setTelemetryValue("LimelightSubsystem Rotation", values[4]);
            TelemetryUpdater.setTelemetryValue("LimelightSubsystem Area", values[2]);
            TelemetryUpdater.setTelemetryValue("Estimated Distance", getDistance());
        }
    }
}