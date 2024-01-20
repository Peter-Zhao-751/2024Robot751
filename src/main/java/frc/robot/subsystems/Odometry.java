package frc.robot.subsystems;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class Odometry extends SwerveDriveOdometry{
    private class states {
        public double position;
        public double velocity;
        public double acceleration;
    
        public states(double p, double v, double a) { 
          this.position = p;
          this.velocity = v;
          this.acceleration = a;
        }
        public states(double p){
            this(p, 0, 0);
        }
        public double getPosition() {
          return position;
        }
        public void reset(double p){
            this.position = p;
            this.velocity = 0;
            this.acceleration = 0;
        }
    }

    private static final double maxLimeTimeout = 0.5;
    private states robotX;
    private states robotY; // no robot theta since pigeon already uses a Kalman filter
    private states limelightX;
    private states limelightY;

    private double previousTime = System.currentTimeMillis();

    private double limeTimeout = 0.0;


    public Odometry(Pose2d limelightPosition, SwerveDriveKinematics kinematics, Rotation2d angle, SwerveModulePosition[] modulePositions){
        super(kinematics, angle, modulePositions);
        robotX = new states(limelightPosition.getX(), kinematics.toChassisSpeeds().vxMetersPerSecond, 0);
        robotY = new states(limelightPosition.getY(), kinematics.toChassisSpeeds().vyMetersPerSecond, 0);

        limelightX = new states(limelightPosition.getX());
        limelightY = new states(limelightPosition.getY());

        resetPosition(angle, modulePositions, new Pose2d(robotX.position, robotY.position, angle));
    }

    public Pose2d update(Pose2d limelightData, Rotation2d angle, SwerveModulePosition[] modulePositions){
        Pose2d pose = super.update(angle, modulePositions);
        updateAll(pose, limelightData);
        updateSwerveOdometry(pose, modulePositions);
        return pose;
    }
    @Override
    public Pose2d update(Rotation2d angle, SwerveModulePosition[] modulePositions){
        Pose2d pose = super.update(angle, modulePositions);
        updateAll(pose);
        updateSwerveOdometry(pose, modulePositions);
        return pose;
    }

    private double linearInterpolation(double pastDisplacement, double pastVelocity, double pastAcc, double sensorDisplacement, double bias, double deltaTime) {
        double predictedDisplacement = pastDisplacement + pastVelocity * deltaTime + 0.5 * pastAcc * deltaTime * deltaTime;
        return predictedDisplacement * (1-bias) + bias * sensorDisplacement;
    }

    private void updateLerp(states state, double sensorPosition, double deltaTime){
        double newPosition = linearInterpolation(state.position, state.velocity, state.acceleration, sensorPosition, 0.5, deltaTime);
        double newVelocity = (newPosition - state.position) / deltaTime;
        state.acceleration = (newVelocity - state.velocity) / deltaTime;
        state.velocity = newVelocity;
        state.position = newPosition;
    }
    
     private void updateAll(Pose2d robotPosition, Pose2d limelightData){
        double deltaTime = getDeltaTime();
        limeTimeout = 0;

        //TODO: tune bias 
        double robotXPosition = robotPosition.getX()*0.2 + limelightData.getX()*0.8;
        double robotYPosition = robotPosition.getY()*0.2 + limelightData.getY()*0.8;
        updateLerp(robotX, robotXPosition, deltaTime);
        updateLerp(robotY, robotYPosition, deltaTime);
        updateLerp(limelightX, robotXPosition, deltaTime);
        updateLerp(limelightY, robotYPosition, deltaTime);
    }

    @Override 
    public Pose2d getPoseMeters(){
        return new Pose2d(robotX.getPosition(), robotY.getPosition(), super.getPoseMeters().getRotation());
    }

    private void updateSwerveOdometry(Pose2d newRobotPosition, SwerveModulePosition[] modulePositions){
        super.resetPosition(newRobotPosition.getRotation(), modulePositions, newRobotPosition);
    }

    private void updateAll(Pose2d robotPosition){
        double deltaTime = getDeltaTime();
        limeTimeout += deltaTime;

        if (limeTimeout >= maxLimeTimeout){
            limelightX.reset(0);
            limelightY.reset(0);
        }

        updateLerp(robotX, robotPosition.getX(), deltaTime);
        updateLerp(robotY, robotPosition.getY(), deltaTime);
    }

    private double getDeltaTime(){
        double currentTime = System.currentTimeMillis();
        previousTime = currentTime;
        return currentTime - previousTime;
    }
}
