package frc.robot.utility;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.math.geometry.Rotation2d;

import com.ctre.phoenix6.hardware.Pigeon2;
import frc.robot.Constants;
import frc.robot.subsystems.LimelightSubsystem;


public class StateEstimator {
    private static StateEstimator instance;

    public Pigeon2 gyro;
    private final LimelightSubsystem limelightSubsystem;
	private final KalmanFilter kalmanFilter;

	private double previousLimelightUpdateTime;
	private Pose2d previousLimelightPose;

	private final ShuffleboardTab tab = Shuffleboard.getTab("Robot State Estimator"); // TODO: look at starting POS x, y and theta and set the values. Start the robot looking at a tag and run auto
   	private final GenericEntry startingPoseX = tab.add("StartingPoseX", 0).getEntry();
	private final GenericEntry startingPoseY = tab.add("StartingPoseY", 0).getEntry();
	private final GenericEntry startingPoseTheta = tab.add("StartingPoseTheta", 0).getEntry();

    public static StateEstimator getInstance() {
        if (instance == null) instance = new StateEstimator();
        return instance;
    }

    private StateEstimator() {
        this.limelightSubsystem = LimelightSubsystem.getInstance();
		this.kalmanFilter = new KalmanFilter(0, 0, 0, 0, 0, 0, Constants.Odometry.kPositionNoiseVar,
				Constants.Odometry.kVelocityNoiseVar, Constants.Odometry.kAccelerationNoiseVar,
				Constants.Odometry.kPositionProcessNoise, Constants.Odometry.kVelocityProcessNoise,
				Constants.Odometry.kAccelerationProcessNoise);
		previousLimelightPose = null;
		previousLimelightUpdateTime = System.currentTimeMillis();
    }

    public void update(SwerveModuleState[] moduleStates){
        double accelerationX = Units.MetersPerSecondPerSecond.convertFrom(gyro.getAccelerationX().getValue(), Units.Gs);
        double accelerationY = Units.MetersPerSecondPerSecond.convertFrom(gyro.getAccelerationY().getValue(), Units.Gs);
        double accelerationZ = Units.MetersPerSecondPerSecond.convertFrom(gyro.getAccelerationZ().getValue(), Units.Gs);

        double quatW = gyro.getQuatW().getValue();
        double quatX = gyro.getQuatX().getValue();
        double quatY = gyro.getQuatY().getValue();
        double quatZ = gyro.getQuatZ().getValue();

        double[][] rotationMatrix = new double[3][3];

        rotationMatrix[0][0] = 1.0 - 2.0 * (quatY * quatY) - 2.0 * (quatZ * quatZ);
        rotationMatrix[0][1] = 2.0 * (quatX * quatY) - 2.0 * (quatW * quatZ);
        rotationMatrix[0][2] = 2.0 * (quatX * quatZ) + 2.0 * (quatW * quatY);

        rotationMatrix[1][0] = 2.0 * (quatX * quatY) + 2.0 * (quatW * quatZ);
        rotationMatrix[1][1] = 1.0 - 2.0 * (quatX * quatX) - 2.0 * (quatZ * quatZ);
        rotationMatrix[1][2] = 2.0 * (quatY * quatZ) - 2.0 * (quatW * quatX);

        rotationMatrix[2][0] = 2.0 * (quatX * quatZ) - 2.0 * (quatW * quatY);
        rotationMatrix[2][1] = 2.0 * (quatY * quatZ) + 2.0 * (quatW * quatX);
        rotationMatrix[2][2] = 1.0 - 2.0 * (quatX * quatX) - 2.0 * (quatY * quatY);


        double fieldAccelerationX = rotationMatrix[0][0] * accelerationX + rotationMatrix[0][1] * accelerationY + rotationMatrix[0][2] * accelerationZ;
        double fieldAccelerationY = rotationMatrix[1][0] * accelerationX + rotationMatrix[1][1] * accelerationY + rotationMatrix[1][2] * accelerationZ;
        double fieldAccelerationZ = rotationMatrix[2][0] * accelerationX + rotationMatrix[2][1] * accelerationY + rotationMatrix[2][2] * accelerationZ;

        TelemetryUpdater.setTelemetryValue("StateEstimator/fieldAccelerationX", fieldAccelerationX);
        TelemetryUpdater.setTelemetryValue("StateEstimator/fieldAccelerationY", fieldAccelerationY);
        TelemetryUpdater.setTelemetryValue("StateEstimator/fieldAccelerationZ", fieldAccelerationZ);
        TelemetryUpdater.setTelemetryValue("Robot Yaw", getYaw().getDegrees());

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
            kalmanFilter.update(fieldChassisSpeedX, fieldChassisSpeedY); // already tested
//            kalmanFilter.updateNewAcceleration(fieldAccelerationX, fieldAccelerationY); // TODO: drive robot and measure if its right
//            kalmanFilter.update(fieldChassisSpeedX, fieldChassisSpeedY, fieldAccelerationX, fieldAccelerationY); // TODO: use this after testing
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

    public void resetPose() {
        setPose(new Pose2d(0, 0, new Rotation2d(0)));
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
        return Rotation2d.fromDegrees((gyro.getYaw().getValue()) % 360);
    }


    public double getVelX(){
        return kalmanFilter.getVelX();
    }
    public double getVelY(){
        return kalmanFilter.getVelY();
    }
}
