package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.math.Conversions;
import frc.robot.Constants;

public class SwerveModule {
    public final int moduleNumber;
    public SwerveModuleState desiredState;
    public final Rotation2d angleOffset;

    private final TalonFX mAngleMotor;
    private final TalonFX mDriveMotor;
    private final CANcoder angleEncoder;

    /* drive motor control requests */
    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0);

    /* angle motor control requests */
    private final PositionVoltage anglePosition = new PositionVoltage(0);

    public SwerveModule(int moduleNumber, Constants.Swerve.SwerveModule moduleConstants) {
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;

        /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.CANCoderID, Constants.CANivoreID);
        CANcoderConfiguration canCoderConfig = SwerveSubsystem.ctreConfigs.swerveCANcoderConfig;
        canCoderConfig.MagnetSensor.MagnetOffset = angleOffset.getRotations();
        angleEncoder.getConfigurator().apply(canCoderConfig);

        /* Angle Motor Config */
        mAngleMotor = new TalonFX(moduleConstants.angleMotorID, Constants.CANivoreID);

        TalonFXConfiguration angleConfiguration = SwerveSubsystem.ctreConfigs.swerveAngleFXConfig;

        angleConfiguration.Feedback.FeedbackRemoteSensorID = angleEncoder.getDeviceID();
        angleConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        angleConfiguration.Feedback.SensorToMechanismRatio = 1.0;
        angleConfiguration.Feedback.RotorToSensorRatio = Constants.Swerve.angleGearRatio;

        angleConfiguration.CurrentLimits.SupplyCurrentLimit = Constants.Swerve.angleCurrentLimit;
        angleConfiguration.CurrentLimits.SupplyTimeThreshold = Constants.Swerve.angleCurrentThreshold;
        angleConfiguration.CurrentLimits.SupplyCurrentLimitEnable = Constants.Swerve.angleEnableCurrentLimit;

        mAngleMotor.getConfigurator().apply(angleConfiguration);
        setAngle();

        anglePosition.EnableFOC = Constants.Swerve.enableFOC;
        driveDutyCycle.EnableFOC = Constants.Swerve.enableFOC;
        driveVelocity.EnableFOC = Constants.Swerve.enableFOC;

        /* Drive Motor Config */
        mDriveMotor = new TalonFX(moduleConstants.driveMotorID, Constants.CANivoreID);

        TalonFXConfiguration driveConfiguration = SwerveSubsystem.ctreConfigs.swerveDriveFXConfig;
        mDriveMotor.getConfigurator().apply(driveConfiguration);
        mDriveMotor.getConfigurator().setPosition(0.0);
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle);
        setAngle(desiredState.angle);
        setSpeed(desiredState, isOpenLoop);
        this.desiredState = desiredState;
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            driveDutyCycle.Output = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            mDriveMotor.setControl(driveDutyCycle);
        } else {
            driveVelocity.Velocity = Conversions.MPSToRPS(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference);
            mDriveMotor.setControl(driveVelocity);
        }
    }

    /**
     * Gets the angle of the module in Rotation2d from the CANCoder
     *
     * @return the angle of the module in Rotation2d
     */
    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValue());
    }

    /**
     * Sets the angle of the module to 0
     * @see SwerveModule#setAngle(double)
     */
    public void setAngle() {
        setAngle(0);
    }

    /**
     * Sets the angle of the module in rotations
     *
     * @param angle the angle to set the module to in rotations
     */
    public void setAngle(double angle) {
        mAngleMotor.setControl(anglePosition.withPosition(angle));
    }

    /**
     * Sets the angle of the module in Rotation2d
     *
     * @param angle the angle to set the module to in Rotation2d
     */

    public void setAngle(Rotation2d angle) {
        setAngle(angle.getRotations());
    }

    /**
     * Get the current state of the module
     *
     * @return the current state of the module in a SwerveModuleState object
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
                Conversions.RPSToMPS(mDriveMotor.getVelocity().getValue(), Constants.Swerve.wheelCircumference),
                Rotation2d.fromRotations(mAngleMotor.getPosition().getValue())
        );
    }

    public double getDriveMotorCurrent() {
        return mDriveMotor.getSupplyCurrent().getValue();
    }

    public double getAngleMotorCurrent() {
        return mAngleMotor.getSupplyCurrent().getValue();
    }

    public double getTotalCurrent() {
        return getDriveMotorCurrent() + getAngleMotorCurrent();
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                Conversions.rotationsToMeters(mDriveMotor.getPosition().getValue(), Constants.Swerve.wheelCircumference),
                Rotation2d.fromRotations(mAngleMotor.getPosition().getValue())
        );
    }

    /**
     * Sets the voltage of the drive motor
     * Used for SysId characterization commands
     *
     * @param voltage the voltage to set the drive motor to
     */
    public void setDriveVoltage(double voltage) {
        mDriveMotor.setVoltage(voltage);
    }

    /**
     * Sets the voltage of the angle motor
     * Used for SysId characterization commands
     *
     * @param voltage the voltage to set the angle motor to
     */
    public void setAngleVoltage(double voltage) {
        mAngleMotor.setVoltage(voltage);
    }
}