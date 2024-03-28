package frc.robot.utility;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.math.geometry.Rotation2d;

import com.ctre.phoenix6.hardware.Pigeon2;
import frc.robot.Constants;
import frc.robot.subsystems.LimelightSubsystem;


public class StateEstimator {
    private final Pigeon2 gyro;
    private final LimelightSubsystem limelightSubsystem;
	private final KalmanFilter kalmanFilter;

	private double previousLimelightUpdateTime;
	private Pose2d previousLimelightPose;

	private final ShuffleboardTab tab = Shuffleboard.getTab("Robot State Estimator"); // TODO: look at starting POS x, y and theta and set the values. Start the robot looking at a tag and run auto
   	private final GenericEntry startingPoseX = tab.add("startingPoseX", 0).getEntry();
	private final GenericEntry startingPoseY = tab.add("startingPoseY", 0).getEntry();
	private final GenericEntry startingPoseTheta = tab.add("startingPoseTheta", 0).getEntry();



    public StateEstimator(Pigeon2 gyro, LimelightSubsystem limelightSubsystem){
        this.gyro = gyro;
        this.limelightSubsystem = limelightSubsystem;
		this.kalmanFilter = new KalmanFilter(0, 0, 0, 0, 0, 0, Constants.Odometry.kPositionNoiseVar,
				Constants.Odometry.kVelocityNoiseVar, Constants.Odometry.kAccelerationNoiseVar,
				Constants.Odometry.kPositionProcessNoise, Constants.Odometry.kVelocityProcessNoise,
				Constants.Odometry.kAccelerationProcessNoise);
		previousLimelightPose = null;
		previousLimelightUpdateTime = System.currentTimeMillis();
    }

    public void update(SwerveModuleState[] moduleStates){
        double accelerationX = gyro.getAccelerationX().getValue();
        double accelerationY = gyro.getAccelerationY().getValue();
        double accelerationZ = gyro.getAccelerationZ().getValue();

        double quatW = gyro.getQuatW().getValue();
        double quatX = gyro.getQuatX().getValue();
        double quatY = gyro.getQuatY().getValue();
        double quatZ = gyro.getQuatZ().getValue();

        double[][] rotationMatrix = new double[3][3];

        rotationMatrix[0][0] = 1.0 - 2.0 * (quatY * quatY + quatZ * quatZ);
        rotationMatrix[0][1] = 2.0 * (quatX * quatY - quatZ * quatW);
        rotationMatrix[0][2] = 2.0 * (quatX * quatZ + quatY * quatW);

        rotationMatrix[1][0] = 2.0 * (quatX * quatY + quatZ * quatW);
        rotationMatrix[1][1] = 1.0 - 2.0 * (quatX * quatX + quatZ * quatZ);
        rotationMatrix[1][2] = 2.0 * (quatY * quatZ - quatX * quatW);

        rotationMatrix[2][0] = 2.0 * (quatX * quatZ - quatY * quatW);
        rotationMatrix[2][1] = 2.0 * (quatY * quatZ + quatX * quatW);
        rotationMatrix[2][2] = 1.0 - 2.0 * (quatX * quatX + quatY * quatY);


        double fieldAccelerationX = rotationMatrix[0][0] * accelerationX + rotationMatrix[0][1] * accelerationY + rotationMatrix[0][2] * accelerationZ;
        double fieldAccelerationY = rotationMatrix[1][0] * accelerationX + rotationMatrix[1][1] * accelerationY + rotationMatrix[1][2] * accelerationZ;
        double fieldAccelerationZ = rotationMatrix[2][0] * accelerationX + rotationMatrix[2][1] * accelerationY + rotationMatrix[2][2] * accelerationZ;

        TelemetryUpdater.setTelemetryValue("fieldAccelerationX", fieldAccelerationX);
        TelemetryUpdater.setTelemetryValue("fieldAccelerationY", fieldAccelerationY);
        TelemetryUpdater.setTelemetryValue("fieldAccelerationZ", fieldAccelerationZ);
        TelemetryUpdater.setTelemetryValue("Robot Yaw", gyro.getYaw().getValue());

        Pose2d newLimePosition = limelightSubsystem.getPose();

        // Chassis speeds
        ChassisSpeeds speeds = Constants.Swerve.swerveKinematics.toChassisSpeeds(moduleStates);

        double fieldChassisSpeedX = rotationMatrix[0][0] * speeds.vxMetersPerSecond + rotationMatrix[0][1] * speeds.vyMetersPerSecond;
        double fieldChassisSpeedY = rotationMatrix[1][0] * speeds.vxMetersPerSecond + rotationMatrix[1][1] * speeds.vyMetersPerSecond;

        //TelemetryUpdater.setTelemetryValue("Field Space Chassis Speeds X", fieldChassisSpeedX);
        //TelemetryUpdater.setTelemetryValue("Field Space Chassis Speeds Y", fieldChassisSpeedY);

        if (limelightSubsystem.hasTarget() && newLimePosition != null) {
			kalmanFilter.update(newLimePosition.getX(), newLimePosition.getY(), fieldChassisSpeedX, fieldChassisSpeedY,
					fieldAccelerationX, fieldAccelerationY);
			previousLimelightPose = newLimePosition;
			previousLimelightUpdateTime = System.currentTimeMillis();
        } else {
            //kalmanFilter.update(fieldChassisSpeedX, fieldChassisSpeedY); // already tested
            kalmanFilter.updateNewAcceleration(fieldAccelerationX, fieldAccelerationY); // TODO: drive robot and measure if its right
            //kalmanFilter.update(fieldChassisSpeedX, fieldChassisSpeedY, fieldAccelerationX, fieldAccelerationY); // TODO: use this after testing
        }

        kalmanFilter.debugDisplayValues();
    }

    public Pose2d getEstimatedPose(){
        return new Pose2d(kalmanFilter.getPosX(), kalmanFilter.getPosY(), new Rotation2d(gyro.getYaw().getValue()));
    }

    public void setPose(Pose2d pose){
        kalmanFilter.reset(pose.getX(), pose.getY(), 0, 0, 0, 0);
        setRotation(pose.getRotation());
    }

	public void setRotation(Rotation2d rotation) {
		gyro.setYaw(rotation.getDegrees());
	}

	public void resetLimelightPose() {
		Pose2d newLimePosition = limelightSubsystem.hasTarget() ? limelightSubsystem.getPose() : previousLimelightPose;
		if (newLimePosition != null || System.currentTimeMillis() - previousLimelightUpdateTime < 400) { // 400 ms of timeout
			newLimePosition = previousLimelightPose;
			kalmanFilter.reset(newLimePosition.getX(), newLimePosition.getY(), 0, 0, 0, 0);
			gyro.setYaw(newLimePosition.getRotation().getDegrees());
		} else {
			kalmanFilter.reset(startingPoseX.getDouble(0), startingPoseY.getDouble(0), 0, 0, 0, 0);
			gyro.setYaw(startingPoseTheta.getDouble(0));
		}
	}

    public Rotation2d getYaw(){ // degrees
        return new Rotation2d(gyro.getYaw().getValue());
    }

    public double getVelX(){
        return kalmanFilter.getVelX();
    }
    public double getVelY(){
        return kalmanFilter.getVelY();
    }
}
