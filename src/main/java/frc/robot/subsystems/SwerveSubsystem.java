package frc.robot.subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;

import frc.lib.util.CTREConfigs;
import frc.robot.Constants;
import frc.robot.utility.StateEstimator;
import frc.robot.utility.TelemetryUpdater;

import static edu.wpi.first.units.Units.Volts;

//import edu.wpi.first.math.estimator.UnscentedKalmanFilter;

// import sendable and sendable buffer


public class SwerveSubsystem extends SubsystemBase {
    public static final CTREConfigs ctreConfigs = new CTREConfigs();
    private static SwerveSubsystem instance;
	private final Field2d m_field = new Field2d();

	private final SwerveDriveOdometry swerveOdometry;
	private final SwerveDrivePoseEstimator poseEstimator;


    private final SwerveModule[] mSwerveMods;
    private final Pigeon2 gyro;

    private final SysIdRoutine routine;

    public boolean lockOnTarget = false;
    private final PIDController angleController = new PIDController(0.1, 0, 0);
    private double desiredAngle = 0;

//    // forgive me father for I have sinned
//    private static GenericEntry resetX, resetY, setButtonEntry;

    private final StateEstimator stateEstimator;

    private SwerveSubsystem() {
        angleController.enableContinuousInput(-180, 180);
        gyro = new Pigeon2(Constants.Swerve.pigeonID, Constants.CANivoreID);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);
//        gyro.setYaw(limelightSubsystem.getYaw()); // TODO: check if this is correct


        mSwerveMods = new SwerveModule[]{
                new SwerveModule(1, Constants.Swerve.frontLeftModule),
                new SwerveModule(2, Constants.Swerve.frontRightModule),
                new SwerveModule(3, Constants.Swerve.backLeftModule),
                new SwerveModule(4, Constants.Swerve.backRightModule)
		};

		Pose2d initPose2d = LimelightSubsystem.getInstance().getPose(); // TODO: i suck

		poseEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions(), initPose2d);

        stateEstimator = StateEstimator.getInstance();
        stateEstimator.gyro = gyro;
        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions());//, new Pose2d());

        TelemetryUpdater.setTelemetryValue("Field", m_field);

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
                        mod.setAngleVoltage(volts.in(Volts));
                    }
                    resetModulesToAbsolute();
                System.out.println("Volts: " + volts.in(Volts));
            }, null, this)
        );

//        resetX = Shuffleboard.getTab("Initializer").add("Reset X", 0).withWidget(BuiltInWidgets.kToggleButton).getEntry();
//        resetY = Shuffleboard.getTab("Initializer").add("Reset Y", 0).withWidget(BuiltInWidgets.kToggleButton).getEntry();
//        setButtonEntry = Shuffleboard.getTab("Initializer").add("Set", false).withWidget(BuiltInWidgets.kToggleButton).getEntry();
    }

    public static SwerveSubsystem getInstance() {
        if (instance == null) instance = new SwerveSubsystem();
        return instance;
    }

    // public Pigeon2 getGyro() {
    //     return gyro;
    // }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction);
    }

    public void setPosition(Pose2d desiredLocation, Rotation2d desiredHeading) {
        drive(null, 0, false, false);
    }

    public boolean drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        return drive(translation, rotation, fieldRelative, isOpenLoop, false);
    }

    public boolean drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop, boolean isPrecise) {
        LimelightSubsystem limelight = LimelightSubsystem.getInstance();
        // precision mode
        double xSpeed = !isPrecise ? translation.getX() : translation.getX() * Constants.Swerve.preciseControlFactor;
        double ySpeed = !isPrecise ? translation.getY() : translation.getY() * Constants.Swerve.preciseControlFactor;
        xSpeed *= Constants.Swerve.speedMultiplier;
        ySpeed *= Constants.Swerve.speedMultiplier;
        double rot = 0;
        if (!lockOnTarget) {
            rot = !isPrecise ? rotation : rotation * Constants.Swerve.preciseControlFactor;
            rot *= Math.max(Constants.Swerve.maxAngularVelocity / 10, 1);
        }
        if (lockOnTarget && limelight.hasTarget()){
            desiredAngle = limelight.getAngle();
        }
        if (lockOnTarget && desiredAngle != 0) {
            rot = angleController.calculate(this.getGyroYaw().getDegrees(), desiredAngle);
            rot = Math.min(rot, 2);
            rot = Math.max(rot, -2);
        }
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

        for (SwerveModule mod : mSwerveMods) mod.setDesiredState(swerveModuleStates[mod.moduleNumber - 1], isOpenLoop);

        return xSpeed >= 0.05 || ySpeed >= 0.05 || rot >= 0.2;
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mSwerveMods) states[mod.moduleNumber - 1] = mod.getState();
        return states;
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) mod.setDesiredState(desiredStates[mod.moduleNumber - 1], false);
    }

    public SwerveModuleState[] getDesiredModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mSwerveMods) states[mod.moduleNumber - 1] = mod.desiredState;
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : mSwerveMods) positions[mod.moduleNumber - 1] = mod.getPosition();
        return positions;
    }

    public Pose2d getPose() {
		return poseEstimator.getEstimatedPosition();//stateEstimator.getEstimatedPose();
    }
    public double getCurrentVelocityMagnitude(){
        return Math.hypot(stateEstimator.getVelX(), stateEstimator.getVelY());
    }

    public void setPose(Pose2d pose) {
        stateEstimator.setPose(pose);
		swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
		poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Pose2d getSwerveOdometryPose2d() {
        return swerveOdometry.getPoseMeters();
    }

	public void setHeading(Rotation2d heading) {
		poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getSwerveOdometryPose2d().getTranslation(), heading));
        stateEstimator.setRotation(heading);
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getSwerveOdometryPose2d().getTranslation(), heading));
    }

    public void zeroHeading() {
        System.out.println("Zeroed Heading");
		setHeading(new Rotation2d(0));
    }

    public Rotation2d getGyroYaw() {
        return stateEstimator.getYaw();
    }

    /**
     * <p> Resets the swerve modules to their absolute positions </p>
     * <p> Align all wheels forwards </p>
     */
    public void resetModulesToAbsolute() {
        System.out.println("Resetting Modules");
        for (SwerveModule mod : mSwerveMods) {
            mod.setAngle();
        }
    }

    /**
     * <p> Crosses the swerve modules </p>
     * <p> Sets the desired state of each module to a 0 speed and a offset rotation of 45 degrees </p>
     * <p> used to prevent defense </p>
     */
    public void crossWheels() {
        System.out.println("Crossing Wheels");
        mSwerveMods[0].setAngle(0.125);
        mSwerveMods[1].setAngle(0.375);
        mSwerveMods[2].setAngle(0.875);
        mSwerveMods[3].setAngle(0.625);
    }

    public void kalmanReset(){
        stateEstimator.resetLimelightPose();
    }

    @Override
    public void periodic() {
        swerveUi();

        swerveOdometry.update(getGyroYaw(), getModulePositions());
		stateEstimator.update(getModuleStates());
		poseEstimator.update(getGyroYaw(), getModulePositions());

        //TelemetryUpdater.setTelemetryValue("total swerve current draw", totalCurrent);

        //CurrentManager.updateCurrent(totalCurrent, CurrentManager.Subsystem.DriveTrain);
    }

    /**
     * <p> Updates the SmartDashboard with the current state of the swerve drive </p>
     */
    private void swerveUi() {

        TelemetryUpdater.setTelemetryValue("Swerve/Swerve X", swerveOdometry.getPoseMeters().getX());
		TelemetryUpdater.setTelemetryValue("Swerve/Swerve Y", swerveOdometry.getPoseMeters().getY());

		TelemetryUpdater.setTelemetryValue("PoseEstimator/Swerve X", poseEstimator.getEstimatedPosition().getX());
		TelemetryUpdater.setTelemetryValue("PoseEstimator/Swerve Y", poseEstimator.getEstimatedPosition().getY());

        TelemetryUpdater.setTelemetryValue("Swerve/Angles/FL Angle", (mSwerveMods[0].getPosition().angle.getDegrees() + 360) % 360);
        TelemetryUpdater.setTelemetryValue("Swerve/Angles/FR Angle", (mSwerveMods[1].getPosition().angle.getDegrees() + 360) % 360);
        TelemetryUpdater.setTelemetryValue("Swerve/Angles/BL Angle", (mSwerveMods[2].getPosition().angle.getDegrees() + 360) % 360);
        TelemetryUpdater.setTelemetryValue("Swerve/Angles/BR Angle", (mSwerveMods[3].getPosition().angle.getDegrees() + 360) % 360);

        LimelightSubsystem limelight = LimelightSubsystem.getInstance();
        if (limelight.hasTarget()) TelemetryUpdater.setTelemetryValue("Limelight/Distance to Target", limelight.getDistance());
        else TelemetryUpdater.setTelemetryValue("Limelight/Distance to Target", Double.NaN);

        //TelemetryUpdater.setTelemetryValue("Robot Pitch", gyro.getPitch().getValue());
        //TelemetryUpdater.setTelemetryValue("Robot Roll", gyro.getRoll().getValue());

        m_field.setRobotPose(getPose()); // TODO: Might be the thing that lags the command scheduler
    }
}
