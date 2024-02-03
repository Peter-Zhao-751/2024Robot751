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
            reset(p, false);
        }
        public void reset(double p, boolean hard){
            this.position = p;
            if (hard){
                this.velocity = 0;
                this.acceleration = 0;
            }
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
        super(kinematics, angle, modulePositions, limelightPosition);
        robotX = new states(limelightPosition.getX(), 0, 0);
        robotY = new states(limelightPosition.getY(), 0, 0);

        limelightX = new states(limelightPosition.getX());
        limelightY = new states(limelightPosition.getY());
    }

    public Odometry(SwerveDriveKinematics kinematics, Rotation2d angle, SwerveModulePosition[] modulePositions){
        this(new Pose2d(0, 0, new Rotation2d(0)), kinematics, angle, modulePositions);
    }

    public Pose2d update(Pose2d limelightData, Rotation2d angle, SwerveModulePosition[] modulePositions){
        Pose2d pose = super.update(angle, modulePositions);

        Pose2d oldPose = getPoseMeters();
        if (limelightData.getX() - oldPose.getX() > 0.1 || limelightData.getY() - oldPose.getY() > 0.1){
            updateSwerveOdometry(limelightData, modulePositions);
            resetStates(limelightData, false);
        }

        updateAll(pose, limelightData);
        updateSwerveOdometry(pose, modulePositions);
        return getPoseMeters();
    }

    @Override
    public Pose2d update(Rotation2d angle, SwerveModulePosition[] modulePositions){
        Pose2d pose = super.update(angle, modulePositions);
        updateAll(pose);
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

        if (limeTimeout >= maxLimeTimeout){
            limelightX.reset(limelightData.getX(), true);
            limelightY.reset(limelightData.getY(), true);
        }else{
            updateLerp(limelightX, limelightData.getX(), deltaTime);
            updateLerp(limelightY, limelightData.getY(), deltaTime);
        }
        limeTimeout = 0;

        double robotXPosition = robotPosition.getX()*0.2 + limelightX.getPosition()*0.8;
        double robotYPosition = robotPosition.getY()*0.2 + limelightY.getPosition()*0.8;
        updateLerp(robotX, robotXPosition, deltaTime);
        updateLerp(robotY, robotYPosition, deltaTime);
    }

    private void updateAll(Pose2d robotPosition){
        double deltaTime = getDeltaTime();
        limeTimeout += deltaTime;

        updateLerp(robotX, robotPosition.getX(), deltaTime);
        updateLerp(robotY, robotPosition.getY(), deltaTime);
    }

    @Override 
    public Pose2d getPoseMeters(){
        return new Pose2d(robotX.getPosition(), robotY.getPosition(), super.getPoseMeters().getRotation());
    }

    private void updateSwerveOdometry(Pose2d newRobotPosition, SwerveModulePosition[] modulePositions){
        super.resetPosition(newRobotPosition.getRotation(), modulePositions, newRobotPosition);
    }

    private void resetStates(Pose2d limePose, boolean hard){
        robotX.reset(limePose.getX(), hard);
        robotY.reset(limePose.getY(), hard);
        limelightX.reset(limePose.getX(), hard);
        limelightY.reset(limePose.getY(), hard);
    }

    private double getDeltaTime(){
        double currentTime = System.currentTimeMillis();
        double deltaTime = currentTime - previousTime;
        previousTime = currentTime;
        return deltaTime;
    }
}
