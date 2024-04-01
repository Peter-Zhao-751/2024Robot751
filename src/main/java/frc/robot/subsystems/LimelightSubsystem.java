package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
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

    private final SwerveSubsystem s_Swerve;

    public static LimelightSubsystem getInstance() {
        if (instance == null) instance = new LimelightSubsystem();
        return instance;
    }

	private LimelightSubsystem() {
        name = Constants.Limelight.name;
        s_Swerve = SwerveSubsystem.getInstance();
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
        Pose2d pose = this.getPose();

        if (pose == null) {
            System.err.println("No target found");
            return 0;
        }

        Constants.FieldConstants.FieldElements redSpeaker = Constants.FieldConstants.red[1];
        Constants.FieldConstants.FieldElements blueSpeaker = Constants.FieldConstants.blue[1];

        Constants.FieldConstants.FieldElements closestSpeaker = Math.hypot(redSpeaker.x - redSpeaker.y, pose.getX() - pose.getY()) <
                Math.hypot(blueSpeaker.x - blueSpeaker.y, pose.getX() - pose.getY()) ? redSpeaker : blueSpeaker;
        double desired = Units.feetToMeters(10);
        double distance = Math.hypot(closestSpeaker.x - pose.getX(), closestSpeaker.y - pose.getY());
        Pose2d desiredPosition = s_Swerve.getPose().plus(new Transform2d(new Translation2d(0, desired - distance), Rotation2d.fromDegrees(this.getAngle())));

        TelemetryUpdater.setTelemetryValue("autoaim/dist/distance", distance);
        TelemetryUpdater.setTelemetryValue("autoaim/dist/x", desiredPosition.getX());
        TelemetryUpdater.setTelemetryValue("autoaim/dist/y", desiredPosition.getY());
        return distance;
    }

    public double getAngle() {
        Pose2d pose = this.getPose();

        if (pose == null) {
            System.err.println("No target found");
            return 0;
        }

        // if the 2 blue field element is closer than the 2 red, then use the blue field elements, otherwise use the red field elements
        Constants.FieldConstants.FieldElements redSpeaker = Constants.FieldConstants.red[1];
        Constants.FieldConstants.FieldElements blueSpeaker = Constants.FieldConstants.blue[1];

        Constants.FieldConstants.FieldElements closestSpeaker = Math.hypot(redSpeaker.x - redSpeaker.y, pose.getX() - pose.getY()) <
         		Math.hypot(blueSpeaker.x - blueSpeaker.y, pose.getX() - pose.getY()) ? redSpeaker : blueSpeaker;
        TelemetryUpdater.setTelemetryValue("aimbot/pose/x", pose.getX());
        TelemetryUpdater.setTelemetryValue("aimbot/pose/y", pose.getY());
        TelemetryUpdater.setTelemetryValue("aimbot/speaker/x", closestSpeaker.x);
        TelemetryUpdater.setTelemetryValue("aimbot/speaker/y", closestSpeaker.y);

        // speaker: 15.76, 5.28
        // us: 12.5, 6
        double angle = (Math.toDegrees(Math.atan2(closestSpeaker.y - pose.getY(), closestSpeaker.x - pose.getX())) + 360) % 360;
        angle = angle > 180 ? angle - 360: angle;
        // angle - pose2d.getRotation().getDegrees() + 360) % 360;
        double delta = this.getYaw() - angle;

        double desiredAngle = (s_Swerve.getGyroYaw().getDegrees() + 360 - delta) % 360;
        TelemetryUpdater.setTelemetryValue("aimbot/speakerAngle", angle);
        TelemetryUpdater.setTelemetryValue("aimbot/limelightYaw", -this.getYaw());
        TelemetryUpdater.setTelemetryValue("aimbot/delta", delta);
        return desiredAngle;
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