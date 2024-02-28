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

public class Constants {

    // Everything on the drive train is on the CANivore, so we use this ID for all CTRE configs
    public static String CANivoreID = "2003 Nissan Ultima";
    public static double stickDeadband = 0.1;

    public static class CurrentManager{
        public static double maxCurrent = 180.0;
        public static double maxPercent = 0.8;
        public static double nominalPercent = 0.5;
    }

    public static class Odometry{
        public static double maxLimeTimeout = 0.5; // seconds
        public static double maxLimeSwerveDeviation = 0.1; // meters
        public static double kalmanGain = 0.5; // bro guess too lazy
        public static double limeSwerveMixRatio = 0.8; // 80% limelight, 20% swerve


        public static double kPositionNoiseVar = 0.075;
        public static double kVelocityNoiseVar = 0.025;
        public static double kAccelerationNoiseVar = 0.25; 

        public static double kPositionProcessNoise = 0.015;
        public static double kVelocityProcessNoise = 0.075;
        public static double kAccelerationProcessNoise = 0.15;
    }

    public static class Shooter{
        // 2 krakens for shooting, one neo for the transfer belts. 
        public static int leftShooterMotorID = 51;
        public static int rightShooterMotorID = 52;

        public static double spinUpTime = 2.0;
        public static double transferSpeed = 0.3;
        public static double feedTime = 0.2;

        public static double kSFlyWheelFeedforward = 0.25;
        public static double kVFlyWheelFeedforward = 0.12;
        public static double kAFlyWheelFeedforward = 0.01;

        public static double kPFlyWheelController = 0.11;
        public static double kIFlyWheelController = 0.0;
        public static double kDFlyWheelController = 0.0;

        public static double kProcessNoise = 3.0;
        public static double kMeasurementNoise = 0.01;
    }

    public static class Intake{
        // 1 falcon for intake, 2 NEOs for moving intake TBD gear ratio, 1 neo for transporting the game piece
        public static int intakeMotorID = 56;
        public static int leftSwivelMotorID = 57;
        public static int rightSwivelMotorID = 58;
        public static int encoderID = 0;

        public static double swivelGearRatio = 15.0; 
        public static double maxSwivelSpeed = 0.15;
        public static double intakeTime = 3.0;

        public static double kPSwivelController = 0.25; // TODO: #5 Using SYSID find the correct PID values for the intake arm pivot
        public static double kISwivelController = 0.0;
        public static double kDSwivelController = 0.0;

        public static double kSwivelTime = 0.17;

        public static double kSSwivelFeedforward = 0.19; // TODO: find this
        public static double kVSwivelFeedforward = 1.17;
        public static double kASwivelFeedforward = 0.0;
        
        public static double kSwivelExtendedAngle = 0.0;  
        public static double kSwivelRetractedAngle = 90.0;
        public static double kSwivelMaintenanceAngle = 45.0;

        public static double kSwivelencoderOffset = 0.0; // TODO: find this
    }

    public static class Transfer{
        public static int shooterTransferID = 53;
        public static double shooterTransferRadius = 2.8575; // units in centimeters

        public static int intakeTransferID = 59;
        public static double intakeTransferRadius = 5.08; // units in centimeters

        public static int beamBreakID = 0;

        public static double kPIntakeController = 1.0;
        public static double kPShooterController = 1.0;

        public static double feedSpeed = 30; // units in centimeters per second

        public static double transferSystemLength = 50; // units in centimeters

        public static double maxTransferTime = 3.0; // unit in seconds
    }

    public static class Climber{
        public static int leftClimberMotorID = 61;
        public static int rightClimberMotorID = 62;
        
        public static double intakeTime = 3.0;

        public static double climberSpringThickness = 0.5; // units in centimeters
        public static double climberSpeed = 0.0;
    }

    public static class Swerve {
        public static int pigeonID = 1;
        public static boolean enableFOC = true;

        public static COTSTalonFXSwerveConstants chosenModule = COTSTalonFXSwerveConstants.SDS.MK4i.KrakenX60(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L3);

        /* Drivetrain Constants */
        public static double trackWidth = 0.55245; //21.75 inches // 27 (Frame width) - 2*2.625 (SDS wheel offset)
        public static double wheelBase = 0.55245;
        public static double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)
        );

        /* Module Gear Ratios */
        public static double driveGearRatio = chosenModule.driveGearRatio;
        public static double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */         
        public static InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static SensorDirectionValue CANCoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static int angleCurrentLimit = 30;
        public static int angleCurrentThreshold = 40;
        public static double angleCurrentThresholdTime = 0.1;
        public static boolean angleEnableCurrentLimit = true;

        public static int driveCurrentLimit = 40; 
        public static int driveCurrentThreshold = 50;
        public static double driveCurrentThresholdTime = 0.1;
        public static boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static double openLoopRamp = 0.25;
        public static double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        // KP is changed below in Swerve Profiling Values
        public static double angleKI = chosenModule.angleKI;
        public static double angleKD = chosenModule.angleKD;

        /* Drive Motor PID Values */
        public static double driveKP = 0.12; //TODO: #4 Tune Drivetrain PID Values
        public static double driveKI = 0.0;
        public static double driveKD = 0.0;
        public static double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static double driveKS = 0.32; //TODO: #4 Tune Drivetrain PID Values
        public static double driveKV = 1.51; //1.51
        public static double driveKA = 0.27;

        /* Swerve Profiling Values */
        public static double preciseControlFactor = 0.4;
        /** Meters per Second */
        public static double maxSpeed = 4.5; //TODO: testing speed, normal: 4.5
        /** Multiplier */
        public static double speedMultiplier = 0.5; //TODO: testing speed, normal 1.0
        /** Radians per Second */
        public static double maxAngularVelocity = maxSpeed / 1.5; // THIS IS THE MAX SPIN SPEED ROBOT, tested 2.1, feels sluggish

        /* Modifier for rotating to desired angle pose speed */
        public static double angleKP = (10.46 * Math.exp(1.05 * speedMultiplier)); // I dont think this is right, take from SYSID instead

        /* Neutral Modes */
        public static NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */

        public static class SwerveModule {
            public int driveMotorID;
            public int angleMotorID;
            public int CANCoderID;
            public Rotation2d angleOffset;
            public SwerveModuleConstants constants;

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
        public static SwerveModule frontLeftModule = new SwerveModule(11, 12, 13, -82.177);
        public static SwerveModule frontRightModule = new SwerveModule(21, 22, 23, -60.292);
        public static SwerveModule backLeftModule = new SwerveModule(31, 32, 33, -79.365);
        public static SwerveModule backRightModule = new SwerveModule(41, 42, 43, 97.910);
    }

    public static class AutoConstants { 
        //TODO: tune everything here
        public static double kMaxSpeedMetersPerSecond = 0.5; // 3 
        public static double kMaxAccelerationMetersPerSecondSquared = 0.5; // 3
        public static double kMaxAngularSpeedRadiansPerSecond = 1; //Math.PI;
        public static double kMaxAngularSpeedRadiansPerSecondSquared = 1; //Math.PI;
        

        //especially these values
        public static double kPXController = 0.8;
        public static double kPYController = 0.8;
        public static double kPThetaController = 1;
    
        /* Constraint for the motion profilied robot angle controller */
        public static TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static class FieldConstants{
        public static class FieldElements{
            public String name;
            public double x;
            public double y;
            public double z;
            
            public double minX;
            public double maxX;
            public double minY;
            public double maxY;

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
        public static FieldElements[] blue = {
            new FieldElements("Amp", 3.57, 7.94, 0.0, -1.0, 1.0, -1.0, 1.0),
            new FieldElements("Speaker", 2.52, 5.3, 0.0, -1.0, 1.0, -1.0, 1.0),
            new FieldElements("Stage", 0, 5, 0.0, -1.0, 1.0, -1.0, 1.0),
            new FieldElements("Chute", 2.8, 0.78, 0.0, -1.0, 1.0, -1.0, 1.0),
        };

        public static FieldElements[] red = {
            new FieldElements("Amp", 12.99, 7.95, 0.0, -1.0, 1.0, -1.0, 1.0),
            new FieldElements("Speaker", 14.02, 5.31, 0.0, -1.0, 1.0, -1.0, 1.0),
            new FieldElements("Stage", 10.74, 4.02, 0.0, -1.0, 1.0, -1.0, 1.0),
            new FieldElements("Chute", 13.74, 0.80, 0.0, -1.0, 1.0, -1.0, 1.0),
        };
    }
    /* Other stuff that I probably need to organize better */
    public static class CANdle {
        public static int CANdleID = 2;
        public static int LEDCount = 21; // TODO: CHANGE TO CORRECT AMOUNT (x = strip + 8)
    }
}
