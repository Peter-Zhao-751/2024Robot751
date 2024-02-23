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

    public static final class Odometry{
        public static final double maxLimeTimeout = 0.5; // seconds
        public static final double maxLimeSwerveDeviation = 0.1; // meters
        public static final double kalmanGain = 0.5; // bro guess too lazy
        public static final double limeSwerveMixRatio = 0.8; // 80% limelight, 20% swerve
    }

    public static final class Shooter{
        // 2 krakens for shooting, one neo for the transfer belts. 
        public static final int leftShooterMotorID = 51;
        public static final int rightShooterMotorID = 52;

        public static final double spinUpTime = 2.0;
        public static final double transferSpeed = 0.3;
        public static final double feedTime = 0.2;
    }

    public static final class Intake{
        // 1 falcon for intake, 2 NEOs for moving intake TBD gear ratio, 1 neo for transporting the game piece
        public static final int intakeMotorID = 56;
        public static final int leftSwivelMotorID = 57;
        public static final int rightSwivelMotorID = 58;
        public static final int encoderID = 0;

        public static final double swivelGearRatio = 15.0; 
        public static final double maxSwivelSpeed = 0.15;
        public static final double intakeTime = 3.0;

        public static final double kPSwivelController = 0.8; // TODO: #5 Using SYSID find the correct PID values for the intake arm pivot
        public static final double kISwivelController = 0.0;
        public static final double kDSwivelController = 0.0;

        public static final double kSwivelExtendedAngle = 0.0; // TODO: #5 Using SYSID find the correct PID values for the intake arm pivot
        public static final double kSwivelRetractedAngle = 90.0; // TODO: #5 Using SYSID find the correct PID values for the intake arm pivot

        public static final double kSwivelencoderOffset = 0.0; // TODO: find this
    }

    public static final class Transfer{
        public static final int shooterTransferID = 53;
        public static final double shooterTransferRadius = 2.8575; // units in centimeters

        public static final int intakeTransferID = 59;
        public static final double intakeTransferRadius = 5.08; // units in centimeters

        public static final int beamBreakID = 0;

        public static final double kPIntakeController = 1.0;
        public static final double kPShooterController = 1.0;

        public static final double feedSpeed = 30; // units in centimeters per second
    }

    public static final class Climber{
        public static final int leftClimberMotorID = 61;
        public static final int rightClimberMotorID = 62;
        
        public static final double intakeTime = 3.0;

        public static final double climberSpringThickness = 0.5; // units in centimeters
        public static final double climberSpeed = 0.0;
    }

    public static final class Swerve {
        public static final int pigeonID = 1;
        public static final boolean enableFOC = true;

        public static final COTSTalonFXSwerveConstants chosenModule = COTSTalonFXSwerveConstants.SDS.MK4i.KrakenX60(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L3);

        /* Drivetrain Constants */
        public static final double trackWidth = 0.55245; //21.75 inches // 27 (Frame width) - 2*2.625 (SDS wheel offset)
        public static final double wheelBase = 0.55245;
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)
        );

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */         
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue CANCoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 30;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 40; 
        public static final int driveCurrentThreshold = 50;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        // KP is changed below in Swerve Profiling Values
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.12; //TODO: #4 Tune Drivetrain PID Values
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.32; //TODO: #4 Tune Drivetrain PID Values
        public static final double driveKV = 1.51; //1.51
        public static final double driveKA = 0.27;

        /* Swerve Profiling Values */
        public static final double preciseControlFactor = 0.4;
        /** Meters per Second */
        public static final double maxSpeed = 4.5; //TODO: testing speed, normal: 4.5
        /** Multiplier */
        public static final double speedMultiplier = 0.5; //TODO: testing speed, normal 1.0
        /** Radians per Second */
        public static final double maxAngularVelocity = maxSpeed / 1.5; // THIS IS THE MAX SPIN SPEED ROBOT, tested 2.1, feels sluggish

        /* Modifier for rotating to desired angle pose speed */
        public static final double angleKP = (10.46 * Math.exp(1.05 * speedMultiplier)); // I dont think this is right, take from SYSID instead

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

        // MAY HAVE TO DO INDIVIDUAL MODULE PID VALUES
        
        //TODO: #6 Tune module angles to be closer bc there are maybe off by a few degrees
        public static final SwerveModule frontLeftModule = new SwerveModule(11, 12, 13, -82.177);
        public static final SwerveModule frontRightModule = new SwerveModule(21, 22, 23, -60.292);
        public static final SwerveModule backLeftModule = new SwerveModule(31, 32, 33, -79.365);
        public static final SwerveModule backRightModule = new SwerveModule(41, 42, 43, 97.910);
    }

    public static final class AutoConstants { 
        //TODO: tune everything here
        public static final double kMaxSpeedMetersPerSecond = 0.5; // 3 
        public static final double kMaxAccelerationMetersPerSecondSquared = 0.5; // 3
        public static final double kMaxAngularSpeedRadiansPerSecond = 1; //Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = 1; //Math.PI;
        

        //especially these values
        public static final double kPXController = 0.8;
        public static final double kPYController = 0.8;
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
        // fix these values
        public static final FieldElements[] blue = {
            new FieldElements("Amp", 3.57, 7.94, 0.0, -1.0, 1.0, -1.0, 1.0),
            new FieldElements("Speaker", 2.52, 5.3, 0.0, -1.0, 1.0, -1.0, 1.0),
            new FieldElements("Stage", 0, 5, 0.0, -1.0, 1.0, -1.0, 1.0),
            new FieldElements("Chute", 2.8, 0.78, 0.0, -1.0, 1.0, -1.0, 1.0),
        };

        public static final FieldElements[] red = {
            new FieldElements("Amp", 12.99, 7.95, 0.0, -1.0, 1.0, -1.0, 1.0),
            new FieldElements("Speaker", 14.02, 5.31, 0.0, -1.0, 1.0, -1.0, 1.0),
            new FieldElements("Stage", 10.74, 4.02, 0.0, -1.0, 1.0, -1.0, 1.0),
            new FieldElements("Chute", 13.74, 0.80, 0.0, -1.0, 1.0, -1.0, 1.0),
        };
    }
    /* Other stuff that I probably need to organize better */
    public static final class CANdle {
        public static final int CANdleID = 2;
        public static final int LEDCount = 21; // TODO: CHANGE TO CORRECT AMOUNT (x = strip + 8)
    }
}
