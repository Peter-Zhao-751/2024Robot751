package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {

    // Everything on the drive train is on the CANivore, so we use this ID for all CTRE configs
    public static final String CANivoreID = "2003 Nissan Ultima";
    public static final double stickDeadband = 0.1;

    public static final class CurrentManager{
        public static final double maxCurrent = 180.0;
        public static final double maxPercent = 0.8;
        public static final double nominalPercent = 0.5;
    }

    public static final class shooter{
        public static final int shooterMotorID = 1;
        public static final int shooterMotorID2 = 2;
        public static final int aimingMotorID = 3;
        public static final double spinUpTime = 2.0;
    }

    public static final class Swerve {
        public static final int pigeonID = 1;

        public static final COTSTalonFXSwerveConstants chosenModule =  //TODO: This must be tuned to specific robot
        COTSTalonFXSwerveConstants.SDS.MK4i.KrakenX60(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L2);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(21.75); // 27 (Frame width) - 2*2.625 (SDS wheel offset)
        public static final double wheelBase = Units.inchesToMeters(21.75);
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */         
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue CANCoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 25;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 30;
        public static final int driveCurrentThreshold = 50;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.12; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.32; //TODO: This must be tuned to specific robot
        public static final double driveKV = 1.51; //1.51
        public static final double driveKA = 0.27;

        /* Swerve Profiling Values */
        public static final double preciseControlFactor = 0.5;
        /** Meters per Second */
        public static final double maxSpeed = 0.02; //TODO: testing speed, normal: 4.5
        /** Radians per Second */
        public static final double maxAngularVelocity = 1.0; // THIS IS THE MAX SPIN SPEED ROBOT

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */

        public static class SwerveModule {
            public final int driveMotorID;
            public final int angleMotorID;
            public final int CANCoderID;
            public final Rotation2d angleOffset;
            public final SwerveModuleConstants constants;

            public SwerveModule(int driveMotorID, int angleMotorID, int CANCoderID, double moduleAngleOffset){
                this.driveMotorID = driveMotorID;
                this.angleMotorID = angleMotorID;
                this.CANCoderID = CANCoderID;
                this.angleOffset = Rotation2d.fromDegrees(moduleAngleOffset);
                this.constants = new SwerveModuleConstants(driveMotorID, angleMotorID, CANCoderID, angleOffset);
            }
        }

        public static final SwerveModule frontLeftModule = new SwerveModule(11, 12, 13, 280);
        public static final SwerveModule frontRightModule = new SwerveModule(21, 22, 23, -60);
        public static final SwerveModule backLeftModule = new SwerveModule(31, 32, 33, 280.7);
        public static final SwerveModule backRightModule = new SwerveModule(41, 42, 43, 98);
    }

    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class FieldConstants{
        public static class FieldElements{
            public final String name;
            public final double x;
            public final double y;
            public final double z;
            
            public final double minX;
            public final double maxX;
            public final double minY;
            public final double maxY;

            public FieldElements(String name, double x, double y, double z, double minX, double maxX, double minY, double maxY){
                this.name = name;
                this.x = x;
                this.y = y;
                this.z = z;
                this.minX = minX;
                this.maxX = maxX;
                this.minY = minY;
                this.maxY = maxY;
            }

            public Boolean isInFieldElements(Pose2d robotPose){
                return robotPose.getX() > minX && robotPose.getX() < maxX && robotPose.getY() > minY && robotPose.getY() < maxY;
            }
        }
        //TODO: Spawn all actual points
        public static final FieldElements[] all = {
            new FieldElements("Amp", 0.0, 0.0, 0.0, -1.0, 1.0, -1.0, 1.0),
            new FieldElements("Speaker", 0.0, 0.0, 0.0, -1.0, 1.0, -1.0, 1.0),
            new FieldElements("Stage", 0.0, 0.0, 0.0, -1.0, 1.0, -1.0, 1.0),
            new FieldElements("Chute", 0.0, 0.0, 0.0, -1.0, 1.0, -1.0, 1.0),
        };
    }
    /* Other stuff that i probably need to organize better */
    public static final class CANdle{
        public static final int CANdleID = 2;
        public static final int LEDCount = 0; // TODO: CHANGE TO CORRECT AMOUNT
    }

}
