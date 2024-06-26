package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import frc.lib.util.LimelightHelpers;
import frc.robot.Constants.FieldConstants.FieldElements;
import frc.robot.utility.TelemetryUpdater;
import frc.lib.util.LimelightHelpers.PoseEstimate;

import java.util.Optional;

public class LimelightSubsystem extends SubsystemBase {
    public enum LEDMode {
		PIPELINE,
		OFF,
		BLINK,
		ON
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

	public PoseEstimate getPoseEstimate() {
        PoseEstimate pose = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
        TelemetryUpdater.setTelemetryValue("Limelight/Est Yaw", pose.pose.getRotation().getDegrees());
		return pose;
	}
    /**
     * 0 is x, 1 is y, 2 z, 3 rotation x, 4 rotation y, 5 rotation z
     * @return Pose2d, null if no target
     */
	public Pose2d getPose() {
		double[] values = getValues();
        return values != null ? new Pose2d(values[0], values[1], new Rotation2d(getYaw())) : null;
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

    private FieldElements closestSpeaker(Pose2d pose) {
        FieldElements redSpeaker = Constants.FieldConstants.red[1];
        FieldElements blueSpeaker = Constants.FieldConstants.blue[1];
        // if the 2 blue field element is closer than the 2 red, then use the blue field elements, otherwise use the red field elements

        return Math.hypot(redSpeaker.x - pose.getX(), redSpeaker.y - pose.getY()) <
                Math.hypot(blueSpeaker.x - pose.getX(), blueSpeaker.y - pose.getY()) ? redSpeaker : blueSpeaker;
        //return redSpeaker;
    }

    private FieldElements closestSpeaker() {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue ? Constants.FieldConstants.blue[1] : Constants.FieldConstants.red[1];
    }

    public double getDistance() {
        Pose2d pose = this.getPose();

        if (pose == null) {
            System.err.println("No target found");
            return 0;
        }

        FieldElements closestSpeaker = closestSpeaker(pose);
        double distance = Math.hypot(closestSpeaker.x - pose.getX(), closestSpeaker.y - pose.getY());

        TelemetryUpdater.setTelemetryValue("autoaim/dist/distance", distance);
        return distance;
    }

    public double getAngle() {
        Pose2d pose = this.getPose();

        if (pose == null) {
            System.err.println("No target found");
            return 0;
        }

        FieldElements closestSpeaker = closestSpeaker(pose);
        TelemetryUpdater.setTelemetryValue("aimbot/pose/x", pose.getX());
        TelemetryUpdater.setTelemetryValue("aimbot/pose/y", pose.getY());
        TelemetryUpdater.setTelemetryValue("aimbot/speaker/x", closestSpeaker.x);
        TelemetryUpdater.setTelemetryValue("aimbot/speaker/y", closestSpeaker.y);

        // speaker: 15.76, 5.28
        // us: 12.5, 6
        double angle = (Math.toDegrees(Math.atan2(closestSpeaker.y - pose.getY(), closestSpeaker.x - pose.getX())) + 360) % 360;
        TelemetryUpdater.setTelemetryValue("aimbot/rawSpeakerAngle", angle);
        angle = angle > 180 ? angle - 360: angle;
        angle = angle < -180 ? angle + 360 : angle;
        double delta = this.getYaw() - angle;

        double desiredAngle = (SwerveSubsystem.getInstance().getGyroYaw().getDegrees() % 360 + 360 - delta) % 360;
        TelemetryUpdater.setTelemetryValue("aimbot/speakerAngle", angle);
        TelemetryUpdater.setTelemetryValue("aimbot/limelightYaw", -this.getYaw());
        TelemetryUpdater.setTelemetryValue("aimbot/delta", delta);
        return desiredAngle;
    }

    @Override
    public void periodic() {
        TelemetryUpdater.setTelemetryValue("Limelight/Limelight Has Target", hasTarget());
        double[] values = getValues();
        if (values != null) {
            TelemetryUpdater.setTelemetryValue("Limelight/Limelight X Position", values[0]);
			TelemetryUpdater.setTelemetryValue("Limelight/Limelight Y Position", values[1]);
			TelemetryUpdater.setTelemetryValue("Limelight/Limelight Rotation", getYaw());
            TelemetryUpdater.setTelemetryValue("Limelight/Estimated Distance", getDistance());
        }
    }
}