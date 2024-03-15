package frc.robot.subsystems;

import frc.lib.util.CTREConfigs;
import frc.robot.Constants;
import frc.robot.utility.KalmanFilter;
import frc.robot.utility.Odometry;
import frc.robot.utility.TelemetryUpdater;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//import com.ctre.phoenix6.SignalLogger;
//import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
//import edu.wpi.first.units.Measure;
//import edu.wpi.first.units.Voltage;
//import static edu.wpi.first.units.Units.Volts;

//import edu.wpi.first.math.estimator.UnscentedKalmanFilter;

// import sendable and sendable buffer


public class SwerveSubsystem extends SubsystemBase implements Component {
    public static final CTREConfigs ctreConfigs = new CTREConfigs();
    public final StructArrayPublisher<SwerveModuleState> actualPublisher;
    public final StructArrayPublisher<SwerveModuleState> desirePublisher;
    private final Field2d m_field = new Field2d();
    private final SwerveDriveOdometry swerveOdometry;
    private final Odometry odometry;
    private final SwerveModule[] mSwerveMods;
    private final Pigeon2 gyro;
    private final Limelight limelight;
//    private final SysIdRoutine routine;

    private final KalmanFilter kalmanFilter;
    private double allocatedCurrent;
    private double totalCurrent;

    public SwerveSubsystem() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID, Constants.CANivoreID);
        limelight = new Limelight();
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);

        mSwerveMods = new SwerveModule[]{
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
        TelemetryUpdater.setTelemetryValue("Field", m_field);

//        routine = new SysIdRoutine(
//            new SysIdRoutine.Config(
//                null,
//                null,
//                null,
//                (state) -> SignalLogger.writeString("state", state.toString())
//            ),
//            new SysIdRoutine.Mechanism(
//                (Measure<Voltage> volts) -> {
//                    for(SwerveModule mod : mSwerveMods){
//                        mod.setDriveVoltage(volts.in(Volts));
//                    }
//                    resetModulesToAbsolute();
//                System.out.println("Volts: " + volts.in(Volts));
//            }, null, this)
//        );

        kalmanFilter = new KalmanFilter(0, 0, 0, 0, 0, 0, Constants.Odometry.kPositionNoiseVar, Constants.Odometry.kVelocityNoiseVar, Constants.Odometry.kAccelerationNoiseVar, Constants.Odometry.kPositionProcessNoise, Constants.Odometry.kVelocityProcessNoise, Constants.Odometry.kAccelerationProcessNoise);
    }

//    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
//        return routine.quasistatic(direction);
//    }
//
//    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
//        return routine.dynamic(direction);
//    }

    public void setPosition(Pose2d desiredLocation, Rotation2d desiredHeading) {
        drive(null, 0, false, false);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        drive(translation, rotation, fieldRelative, isOpenLoop, false);
    }

    public boolean drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop, boolean isPrecise) {
        // precision mode
        double xSpeed = !isPrecise ? translation.getX() : translation.getX() * Constants.Swerve.preciseControlFactor;
        double ySpeed = !isPrecise ? translation.getY() : translation.getY() * Constants.Swerve.preciseControlFactor;
        xSpeed = xSpeed * Constants.Swerve.speedMultiplier;
        ySpeed = ySpeed * Constants.Swerve.speedMultiplier;
        double rot = !isPrecise ? rotation : rotation * Constants.Swerve.preciseControlFactor;
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

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber - 1], isOpenLoop);
        }

        return (xSpeed >= 0.05 && ySpeed >= 0.05 && rot >= 0.2); // TODO: tune these values
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber - 1], false);
        }
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber - 1] = mod.getState();
        }
        return states;
    }

    public SwerveModuleState[] getDesiredModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber - 1] = mod.desiredState;
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : mSwerveMods) {
            positions[mod.moduleNumber - 1] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void setPose(Pose2d pose) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public void setHeading(Rotation2d heading) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading() {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValue());
    }

    public Pose2d fuseLimelightSwerveData(Pose2d limelightData, Pose2d swerveData) {
        return new Pose2d(); // TODO
    }

    /**
     * <p> Resets the swerve modules to their absolute positions </p>
     * <p> Align all wheels forwards </p>
     */
    public void resetModulesToAbsolute() {
        for (SwerveModule mod : mSwerveMods) {
            mod.resetToAbsolute();
        }
    }

    /**
     * <p> Crosses the swerve modules </p>
     * <p> Sets the desired state of each module to a 0 speed and a offset rotation of 45 degrees </p>
     * <p> used to prevent defense </p>
     */
    public void crossModules() {
        for (SwerveModule mod : mSwerveMods) {
            mod.setModuleAngle((mod.moduleNumber - 1) * 0.25 + 0.125);
        }
    }

    @Override
    public void periodic() {
        swerveUi();

        // update current
        totalCurrent = 0;
        for (SwerveModule mod : mSwerveMods) {
            totalCurrent += mod.getTotalCurrent();
        }
        //TelemetryUpdater.setTelemetryValue("total swerve current draw", totalCurrent);

        //CurrentManager.updateCurrent(totalCurrent, CurrentManager.Subsystem.DriveTrain);
    }

    /**
     * <p> Updates the SmartDashboard with the current state of the swerve drive </p>
     */
    private void swerveUi() {
        double accelerationX = gyro.getAccelerationX().getValue();
        double accelerationY = gyro.getAccelerationY().getValue();
        double accelerationZ = gyro.getAccelerationZ().getValue();

        double yaw = gyro.getYaw().getValue();
        double pitch = gyro.getPitch().getValue();
        double roll = gyro.getRoll().getValue();

        double yawRadians = Math.toRadians(yaw);

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

        swerveOdometry.update(getGyroYaw(), getModulePositions());

        // Experimental odometry fusion using limelight and swerve
        Pose2d newLimePosition = limelight.getPose();

        // Chassis speeds

        ChassisSpeeds speeds = Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());

        double fieldChassisSpeedX = rotationMatrix[0][0] * speeds.vxMetersPerSecond + rotationMatrix[0][1] * speeds.vyMetersPerSecond;
        double fieldChassisSpeedY = rotationMatrix[1][0] * speeds.vxMetersPerSecond + rotationMatrix[1][1] * speeds.vyMetersPerSecond;

        //TelemetryUpdater.setTelemetryValue("Field Space Chassis Speeds X", fieldChassisSpeedX);
        //TelemetryUpdater.setTelemetryValue("Field Space Chassis Speeds Y", fieldChassisSpeedY);

        if (limelight.hasTarget() && newLimePosition != null) {
            kalmanFilter.update(newLimePosition.getX(), newLimePosition.getY(), fieldChassisSpeedX, fieldChassisSpeedY, fieldAccelerationX, fieldAccelerationY);
            odometry.update(newLimePosition, getGyroYaw(), getModulePositions());
        } else {
            odometry.update(getGyroYaw(), getModulePositions());
            //kalmanFilter.update(fieldChassisSpeedX, fieldChassisSpeedY);
            kalmanFilter.update(fieldChassisSpeedX, fieldChassisSpeedY, fieldAccelerationX, fieldAccelerationY);
        }

        kalmanFilter.debugDisplayValues();

        // for(SwerveModule mod : mSwerveMods){
        //     TelemetryUpdater.setTelemetryValue("Mod " + mod.moduleNumber + " Drive Current", mod.getDriveMotorCurrent());
        //     TelemetryUpdater.setTelemetryValue("Mod " + mod.moduleNumber + " Angle Current", mod.getAngleMotorCurrent());
        //     TelemetryUpdater.setTelemetryValue("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
        //     TelemetryUpdater.setTelemetryValue("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
        //     TelemetryUpdater.setTelemetryValue("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        // }

        TelemetryUpdater.setTelemetryValue("swerve x", swerveOdometry.getPoseMeters().getX());
        TelemetryUpdater.setTelemetryValue("swerve y", swerveOdometry.getPoseMeters().getY());

        TelemetryUpdater.setTelemetryValue("Robot Yaw", gyro.getYaw().getValue());

        limelight.debugDisplayValues();
        //TelemetryUpdater.setTelemetryValue("Robot Pitch", gyro.getPitch().getValue());
        //TelemetryUpdater.setTelemetryValue("Robot Roll", gyro.getRoll().getValue());

        //m_field.setRobotPose(odometry.getPoseMeters());

        // TelemetryUpdater.setTelemetryValueata("Swerve Drive", new Sendable() {
        //     @Override
        //     public void initSendable(SendableBuilder builder) {
        //         builder.setSmartDashboardType("SwerveDrive");

        //         builder.addDoubleProperty("Front Left Angle", () -> mSwerveMods[0].getPosition().angle.getRadians(), null);
        //         builder.addDoubleProperty("Front Left Velocity", () -> mSwerveMods[0].getState().speedMetersPerSecond, null);

        //         builder.addDoubleProperty("Front Right Angle", () -> mSwerveMods[1].getPosition().angle.getRadians(), null);
        //         builder.addDoubleProperty("Front Right Velocity", () -> mSwerveMods[1].getState().speedMetersPerSecond, null);

        //         builder.addDoubleProperty("Back Left Angle", () -> mSwerveMods[2].getPosition().angle.getRadians(), null);
        //         builder.addDoubleProperty("Back Left Velocity", () -> mSwerveMods[2].getState().speedMetersPerSecond, null);

        //         builder.addDoubleProperty("Back Right Angle", () -> mSwerveMods[3].getPosition().angle.getRadians(), null);
        //         builder.addDoubleProperty("Back Right Velocity", () -> mSwerveMods[3].getState().speedMetersPerSecond, null);

        //         builder.addDoubleProperty("Robot Angle", () -> getGyroYaw().getRadians(), null);
        //     }
        // });
    }

    @Override
    public void allocateCurrent(double current) {
        //set motor controller current
    }

    @Override
    public double getCurrentDraw() {
        return totalCurrent;
    }

    @Override
    public int getPriority() {
        return 1;
    }
}