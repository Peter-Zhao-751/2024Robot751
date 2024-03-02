package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.utility.KalmanFilter;
import frc.robot.utility.Odometry;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import static edu.wpi.first.units.Units.Volts;
//import edu.wpi.first.math.estimator.UnscentedKalmanFilter;

// import sendable and sendable buffer
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class SwerveSubsystem extends SubsystemBase implements Component{
    public final StructArrayPublisher<SwerveModuleState> actualPublisher;
    public final StructArrayPublisher<SwerveModuleState> desirePublisher;
    private final Field2d m_field = new Field2d();
    public final SwerveDriveOdometry swerveOdometry;
    public final Odometry odometry;
    public final SwerveModule[] mSwerveMods;
    public final Pigeon2 gyro;
    public final Limelight limelight;
    public final SysIdRoutine routine;

    private final KalmanFilter kalmanFilter;
    private double allocatedCurrent;

    public SwerveSubsystem() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID, Constants.CANivoreID);
        limelight = new Limelight();
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(1, Constants.Swerve.frontLeftModule),
            new SwerveModule(2, Constants.Swerve.frontRightModule),
            new SwerveModule(3, Constants.Swerve.backLeftModule),
            new SwerveModule(4, Constants.Swerve.backRightModule)
        };

        //odometry = new Odometry(limelight.getPose(), Constants.Swerve.swerveKinematics, getHeading(), getModulePositions());
        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions());//, new Pose2d());

        odometry = new Odometry(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions());

        actualPublisher = NetworkTableInstance.getDefault().getStructArrayTopic("SwerveActualStates", SwerveModuleState.struct).publish();
        desirePublisher = NetworkTableInstance.getDefault().getStructArrayTopic("SwerveDesiredStates", SwerveModuleState.struct).publish();
        SmartDashboard.putData("Field", m_field);

        routine = new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> SignalLogger.writeString("state", state.toString())
            ), 
            new SysIdRoutine.Mechanism(
                (Measure<Voltage> volts) -> {
                    for(SwerveModule mod : mSwerveMods){
                        mod.setDriveVoltage(volts.in(Volts));
                    }
                System.out.println("Volts: " + volts.in(Volts));
            }, null, this)
        );

        kalmanFilter = new KalmanFilter(0, 0, 0, 0, 0, 0, Constants.Odometry.kPositionNoiseVar, Constants.Odometry.kVelocityNoiseVar, Constants.Odometry.kAccelerationNoiseVar, Constants.Odometry.kPositionProcessNoise, Constants.Odometry.kVelocityProcessNoise, Constants.Odometry.kAccelerationProcessNoise);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction);
    }
    
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction);
    }

    public void setPosition(Pose2d desiredLocation, Rotation2d desiredHeading){
        drive(null, 0, false, false);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        drive(translation, rotation, fieldRelative, isOpenLoop, false);
    }

    public boolean drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop, boolean isPrecise) {
        // precision mode
        Double xSpeed = !isPrecise ? translation.getX(): translation.getX() * Constants.Swerve.preciseControlFactor;
        Double ySpeed = !isPrecise ? translation.getY(): translation.getY() * Constants.Swerve.preciseControlFactor;
        xSpeed = xSpeed * Constants.Swerve.speedMultiplier;
        ySpeed = ySpeed * Constants.Swerve.speedMultiplier;
        Double rot = !isPrecise ? rotation: rotation * Constants.Swerve.preciseControlFactor;
        rot = rot * Math.max(Constants.Swerve.maxAngularVelocity / 10, 1);

        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    xSpeed, 
                                    ySpeed, 
                                    rot, 
                                    getGyroYaw()
                                )
                                : new ChassisSpeeds(
                                    xSpeed, 
                                    ySpeed, 
                                    rot)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber-1], isOpenLoop);
        }

        return (xSpeed >= 0.05 && ySpeed >= 0.05 && rot >= 0.2); // TODO: tune these values
    }

    public void crossWheels() { // TODO: #7 Check cross wheels works
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(new SwerveModuleState(0, new Rotation2d((mod.moduleNumber-1) * 90 + 45)), false);
        }
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber-1], false);
        }
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber-1] = mod.getState();
        }
        return states;
    }

    public SwerveModuleState[] getDesiredModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber-1] = mod.desiredState;
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber-1] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void setPose(Pose2d pose) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public void setHeading(Rotation2d heading){
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading(){
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValue());
    }

    public Pose2d fuseLimelightSwerveData(Pose2d limelightData, Pose2d swerveData){
        return new Pose2d();
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    public void crossModules(){
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(new SwerveModuleState(0, new Rotation2d((mod.moduleNumber-1) * 90 + 45)), false);
        }
    } 

    @Override
    public void periodic(){
        double accelerationX = gyro.getAccelerationX().getValue();
        double accelerationY = gyro.getAccelerationY().getValue();
        double accelerationZ = gyro.getAccelerationZ().getValue();

        double yaw = gyro.getYaw().getValue();
        double pitch = gyro.getPitch().getValue();
        double roll = gyro.getRoll().getValue();
        
        double yawRadians = Math.toRadians(yaw);
        double fieldAccelerationX = accelerationX * Math.cos(yawRadians) - accelerationY * Math.sin(yawRadians);
        double fieldAccelerationY = accelerationX * Math.sin(yawRadians) + accelerationY * Math.cos(yawRadians);

        SmartDashboard.putNumber("traditional fieldAccelerationX", fieldAccelerationX);
        SmartDashboard.putNumber("traditional fieldAccelerationY", fieldAccelerationY);


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

        double[] fieldAcceleration = new double[3];
        fieldAcceleration[0] = rotationMatrix[0][0] * accelerationX + rotationMatrix[0][1] * accelerationY + rotationMatrix[0][2] * accelerationZ;
        fieldAcceleration[1] = rotationMatrix[1][0] * accelerationX + rotationMatrix[1][1] * accelerationY + rotationMatrix[1][2] * accelerationZ;
        fieldAcceleration[2] = rotationMatrix[2][0] * accelerationX + rotationMatrix[2][1] * accelerationY + rotationMatrix[2][2] * accelerationZ;

        SmartDashboard.putNumber("fieldAccelerationX", fieldAcceleration[0]);
        SmartDashboard.putNumber("fieldAccelerationY", fieldAcceleration[1]);
        SmartDashboard.putNumber("fieldAccelerationZ", fieldAcceleration[2]);

        // update current
        double totalCurrent = 0;
        for (SwerveModule mod : mSwerveMods){
            totalCurrent += mod.getTotalCurrent();
        }
        SmartDashboard.putNumber("total swerve current draw", totalCurrent);
        
        //CurrentManager.updateCurrent(totalCurrent, CurrentManager.Subsystem.DriveTrain);

        swerveOdometry.update(getGyroYaw(), getModulePositions());

        // Experimental odometry fusion using limelight and swerve
        Pose2d newLimePosition = limelight.getPose();
        limelight.debugDisplayValues();

        // Chassis speeds

        ChassisSpeeds speeds = Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());

        SmartDashboard.putNumber("Chassis Speeds X", speeds.vxMetersPerSecond);
        SmartDashboard.putNumber("Chassis Speeds Y", speeds.vyMetersPerSecond);


        double fieldChassisSpeedX = rotationMatrix[0][0] * speeds.vxMetersPerSecond + rotationMatrix[0][1] * speeds.vyMetersPerSecond;
        double fieldChassisSpeedY = rotationMatrix[1][0] * speeds.vxMetersPerSecond + rotationMatrix[1][1] * speeds.vyMetersPerSecond;

        SmartDashboard.putNumber("Field Space Chassis Speeds Y", fieldChassisSpeedX);
        SmartDashboard.putNumber("Field Space Chassis Speeds Y", fieldChassisSpeedY);

        if (limelight.hasTarget() && newLimePosition != null){
            kalmanFilter.update(newLimePosition.getX(), newLimePosition.getY(), fieldChassisSpeedX, fieldChassisSpeedY, fieldAcceleration[0], fieldAcceleration[1]);
            odometry.update(newLimePosition, getGyroYaw(), getModulePositions());
        } else {
            odometry.update(getGyroYaw(), getModulePositions());
            kalmanFilter.update(fieldChassisSpeedX, fieldChassisSpeedY, fieldAcceleration[0], fieldAcceleration[1]);
        }

        kalmanFilter.debugDisplayValues();
        
        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Drive Current", mod.getDriveMotorCurrent());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle Current", mod.getAngleMotorCurrent());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }
        
        desirePublisher.set(getDesiredModuleStates());
        
        actualPublisher.set(getModuleStates());

        SmartDashboard.putNumber("robot x", odometry.getPoseMeters().getX());
        SmartDashboard.putNumber("robot y", odometry.getPoseMeters().getY());

        SmartDashboard.putNumber("swerve x", swerveOdometry.getPoseMeters().getX());
        SmartDashboard.putNumber("swerve y", swerveOdometry.getPoseMeters().getY());

        SmartDashboard.putNumber("Robot Yaw", gyro.getYaw().getValue());
        SmartDashboard.putNumber("Robot Pitch", gyro.getPitch().getValue());
        SmartDashboard.putNumber("Robot Roll", gyro.getRoll().getValue());

        m_field.setRobotPose(odometry.getPoseMeters());

        SmartDashboard.putData("Swerve Drive", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType("SwerveDrive");
          
                builder.addDoubleProperty("Front Left Angle", () -> mSwerveMods[0].getPosition().angle.getRadians(), null);
                builder.addDoubleProperty("Front Left Velocity", () -> mSwerveMods[0].getState().speedMetersPerSecond, null);

                builder.addDoubleProperty("Front Right Angle", () -> mSwerveMods[1].getPosition().angle.getRadians(), null);
                builder.addDoubleProperty("Front Right Velocity", () -> mSwerveMods[1].getState().speedMetersPerSecond, null);

                builder.addDoubleProperty("Back Left Angle", () -> mSwerveMods[2].getPosition().angle.getRadians(), null);
                builder.addDoubleProperty("Back Left Velocity", () -> mSwerveMods[2].getState().speedMetersPerSecond, null);

                builder.addDoubleProperty("Back Right Angle", () -> mSwerveMods[3].getPosition().angle.getRadians(), null);
                builder.addDoubleProperty("Back Right Velocity", () -> mSwerveMods[3].getState().speedMetersPerSecond, null);
          
                builder.addDoubleProperty("Robot Angle", () -> getGyroYaw().getRadians(), null);
            }
        });
    }
    @Override
    public void allocateCurrent(double current){
        //set motor controller current
    }
    @Override
    public double getCurrentDraw(){
        return 0;
    }
    @Override
    public int getPriority(){
        return 1;
    }
}