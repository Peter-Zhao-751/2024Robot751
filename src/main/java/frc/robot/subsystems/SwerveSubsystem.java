package frc.robot.subsystems;

import frc.lib.util.CTREConfigs;
import frc.robot.Constants;
import frc.robot.utility.StateEstimator;
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
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
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
    private static SwerveSubsystem instance;
    public final StructArrayPublisher<SwerveModuleState> actualPublisher;
    public final StructArrayPublisher<SwerveModuleState> desirePublisher;
    private final Field2d m_field = new Field2d();
    private final SwerveDriveOdometry swerveOdometry;
    private final SwerveModule[] mSwerveMods;
    private final Pigeon2 gyro;
    private final Limelight limelight;

//    private final SysIdRoutine routine;

    // forgive me father for I have sinned
    private static GenericEntry resetX, resetY, setButtonEntry;

    private final StateEstimator stateEstimator;
    private double allocatedCurrent;

    private SwerveSubsystem() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID, Constants.CANivoreID);
        limelight = Limelight.getInstance();
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);

        mSwerveMods = new SwerveModule[]{
                new SwerveModule(1, Constants.Swerve.frontLeftModule),
                new SwerveModule(2, Constants.Swerve.frontRightModule),
                new SwerveModule(3, Constants.Swerve.backLeftModule),
                new SwerveModule(4, Constants.Swerve.backRightModule)
        };

        stateEstimator = new StateEstimator(gyro, limelight);
        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions());//, new Pose2d());

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
        
        resetX = Shuffleboard.getTab("Initializer").add("Reset X", 0).withWidget(BuiltInWidgets.kToggleButton).getEntry();
        resetY = Shuffleboard.getTab("Initializer").add("Reset Y", 0).withWidget(BuiltInWidgets.kToggleButton).getEntry();
        setButtonEntry = Shuffleboard.getTab("Initializer").add("Set", false).withWidget(BuiltInWidgets.kToggleButton).getEntry();
    }

    public static SwerveSubsystem getInstance() {
        if (instance == null) instance = new SwerveSubsystem();
        return instance;
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

    public boolean drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        return drive(translation, rotation, fieldRelative, isOpenLoop, false);
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

        return (xSpeed >= 0.05 && ySpeed >= 0.05 && rot >= 0.2);
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber - 1] = mod.getState();
        }
        return states;
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber - 1], false);
        }
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
        return stateEstimator.getEstimatedPose();
    }

    public void setPose(Pose2d pose) {
        stateEstimator.setPose(pose);
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Pose2d getSwerveOdometryPose2d() {
        return swerveOdometry.getPoseMeters();
    }

    public void setHeading(Rotation2d heading) {
        stateEstimator.setRotation(heading);
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getSwerveOdometryPose2d().getTranslation(), heading));
    }

    public void zeroHeading() {
        stateEstimator.setRotation(new Rotation2d(0));
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getSwerveOdometryPose2d().getTranslation(), new Rotation2d()));
    }

    public Rotation2d getGyroYaw() {
        return stateEstimator.getYaw();
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
    public void crossWheels() {
        for (SwerveModule mod : mSwerveMods) {
            mod.setModuleAngle((mod.moduleNumber - 1) * 0.25 + 0.125);
        }
    }

    @Override
    public void periodic() {
        swerveUi();

        swerveOdometry.update(getGyroYaw(), getModulePositions());
        stateEstimator.update(getModuleStates());

        //TelemetryUpdater.setTelemetryValue("total swerve current draw", totalCurrent);

        //CurrentManager.updateCurrent(totalCurrent, CurrentManager.Subsystem.DriveTrain);
    }

    /**
     * <p> Updates the SmartDashboard with the current state of the swerve drive </p>
     */
    private void swerveUi() {

        // if (setButtonEntry.getBoolean(false)) {
        //     setPose(new Pose2d(resetX.getDouble(0), resetY.getDouble(0), getGyroYaw()));
        // }

        TelemetryUpdater.setTelemetryValue("swerve x", swerveOdometry.getPoseMeters().getX());
        TelemetryUpdater.setTelemetryValue("swerve y", swerveOdometry.getPoseMeters().getY());

        limelight.debugDisplayValues();
        //TelemetryUpdater.setTelemetryValue("Robot Pitch", gyro.getPitch().getValue());
        //TelemetryUpdater.setTelemetryValue("Robot Roll", gyro.getRoll().getValue());

        m_field.setRobotPose(getPose());


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
        double totalCurrent = 0;
        for (SwerveModule mod : mSwerveMods) {
            totalCurrent += mod.getTotalCurrent();
        }
        return totalCurrent;
    }

    @Override
    public int getPriority() {
        return 1;
    }
}