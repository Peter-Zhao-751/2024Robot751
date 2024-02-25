package frc.robot.subsystems;

import frc.robot.Constants;

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

public class SwerveDrive extends SubsystemBase {
    public final StructArrayPublisher<SwerveModuleState> actualPublisher;
    public final StructArrayPublisher<SwerveModuleState> desirePublisher;
    private final Field2d m_field = new Field2d();
    public final SwerveDriveOdometry swerveOdometry;
    public final Odometry odometry;
    public final SwerveModule[] mSwerveMods;
    public final Pigeon2 gyro;
    public final Limelight limelight;
    public final SysIdRoutine routine;

    public SwerveDrive() {
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

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop, boolean isPrecise) {
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
                                    getHeading()
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
    }

    public void crossWheels() { // TODO: #7 Check cross wheels works
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(new SwerveModuleState(0, new Rotation2d((mod.moduleNumber-1) * Math.PI/2 + Math.PI/4)), false);
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

    public Rotation2d getHeading(){
        return getPose().getRotation();
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

        SmartDashboard.putNumber("fieldAccelerationX", fieldAccelerationX);
        SmartDashboard.putNumber("fieldAccelerationY", fieldAccelerationY);

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

        if (limelight.hasTarget() && newLimePosition != null){
            odometry.update(newLimePosition, getGyroYaw(), getModulePositions());
        } else {
            odometry.update(getGyroYaw(), getModulePositions());
        }
        
        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Drive Current", mod.getDriveMotorCurrent());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle Current", mod.getAngleMotorCurrent());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }
        
        desirePublisher.set(getDesiredModuleStates());
        
        actualPublisher.set(getModuleStates());

        SmartDashboard.putNumber("Robot Angle", getHeading().getDegrees());
        SmartDashboard.putNumber("robot x", odometry.getPoseMeters().getX());
        SmartDashboard.putNumber("robot y", odometry.getPoseMeters().getY());

        SmartDashboard.putNumber("swerve x", swerveOdometry.getPoseMeters().getX());
        SmartDashboard.putNumber("swerve y", swerveOdometry.getPoseMeters().getY());

        m_field.setRobotPose(odometry.getPoseMeters());
    }
}