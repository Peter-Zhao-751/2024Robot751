package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utility.TelemetryUpdater;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static edu.wpi.first.units.Units.Volts;

public class IntakeSubsystem extends SubsystemBase{
    private static IntakeSubsystem instance;

    private final CANSparkMax leftSwivelMotor;
    private final CANSparkMax rightSwivelMotor;

    private final TalonFX intakeMotor;
    private final VelocityVoltage velocityVoltage;

    private final SparkAbsoluteEncoder angleEncoder;
    private final TrapezoidProfile swivelTrapezoidProfile;


    private final ArmFeedforward swivelFeedforwardController;
    private final PIDController swivelPIDController;
    //private final PIDController intakePIDController;
    private final DigitalInput beamBreak;
    private final Debouncer beamDebouncer;
    private boolean isBeamBroken;

    private double swivelSetpoint;
    private double targetIntakeSpeed;
    private double swivelMovementStartTime;
    private double swivelMovementStartAngle;
    private boolean isSwivelEnabled;

    private final SysIdRoutine routine;

    private IntakeSubsystem() {
        leftSwivelMotor = new CANSparkMax(Constants.Intake.leftSwivelMotorID, MotorType.kBrushless);
        rightSwivelMotor = new CANSparkMax(Constants.Intake.rightSwivelMotorID, MotorType.kBrushless);
        intakeMotor = new TalonFX(Constants.Intake.intakeMotorID);

        leftSwivelMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        leftSwivelMotor.setInverted(true);
        rightSwivelMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        angleEncoder = rightSwivelMotor.getAbsoluteEncoder(Type.kDutyCycle);
        angleEncoder.setInverted(true);
        angleEncoder.setPositionConversionFactor(360);
        angleEncoder.setZeroOffset(Constants.Intake.kSwivelEncoderZeroOffset);

        beamBreak = new DigitalInput(Constants.Intake.beamBreakDIOPort);
        beamDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kRising);
        isBeamBroken = false;


        TalonFXConfiguration intakeMotorConfig = new TalonFXConfiguration();
        Slot0Configs slot0 = intakeMotorConfig.Slot0;
        slot0.kS = Constants.Intake.kSIntakeController;
        slot0.kV = Constants.Intake.kVIntakeController;
        slot0.kP = Constants.Intake.kPIntakeController;
        slot0.kI = Constants.Intake.kIIntakeController;
        slot0.kD = Constants.Intake.kDIntakeController;
        intakeMotor.getConfigurator().apply(slot0);

        velocityVoltage = new VelocityVoltage(0);

        swivelPIDController = new PIDController(Constants.Intake.kPSwivelController, Constants.Intake.kISwivelController, Constants.Intake.kDSwivelController);
        swivelPIDController.enableContinuousInput(0, 360);
        swivelFeedforwardController = new ArmFeedforward(Constants.Intake.kSSwivelFeedforward, Constants.Intake.kGSwivelFeedforward, Constants.Intake.kVSwivelFeedforward);

        //intakePIDController = new PIDController(Constants.Intake.kPIntakeController, Constants.Intake.kIIntakeController, Constants.Intake.kDIntakeController);

        swivelTrapezoidProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(300, 600));

        targetIntakeSpeed = 0;

        swivelSetpoint = getSwivelPosition();
        swivelMovementStartTime = System.currentTimeMillis();
        swivelMovementStartAngle = getSwivelPosition();

        isSwivelEnabled = true;

//        slider = Shuffleboard.getTab("intake")
//        .add("Slider", 0)
//        .withWidget(BuiltInWidgets.kNumberSlider)
//        .withProperties(Map.of("min", 0, "max", 12))
//        .getEntry();

        routine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null,
                        null,
                        null,
                        (state) -> SignalLogger.writeString("state", state.toString())
                ),
                new SysIdRoutine.Mechanism(
                        (Measure<Voltage> volts) -> {
                            intakeMotor.setVoltage(volts.in(Volts));
                            System.out.println("Volts: " + volts.in(Volts));
                        }, null, this)
        );
    }

    public static IntakeSubsystem getInstance() {
        if (instance == null) instance = new IntakeSubsystem();
        return instance;
    }

    /**
     * Changing voltage
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction);
    }

    /**
     * Static voltage
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction);
    }

    /**
     * Stops the intake and swivel motors
     */
    public void stopAll() {
        leftSwivelMotor.stopMotor();
        rightSwivelMotor.stopMotor();
        setIntakeSpeed(0);
        // intakeMotor.stopMotor();
    }

    /**
     * Returns the position of the swivel in degrees
     *
     * @return double, the position of the swivel in degrees
     */
    public double getSwivelPosition() {
        double currentAngle = angleEncoder.getPosition() % 360;
        if (currentAngle >= 180) return currentAngle - 360;
        return currentAngle;
    }

    /**
     * Sets the position of the swivel
     *
     * @param position the position of the swivel in degrees
     */
    public void setSwivelPosition(double position) {
        swivelSetpoint = position;
        swivelMovementStartAngle = getSwivelPosition();
        swivelMovementStartTime = System.currentTimeMillis();
    }

    /**
     * Returns the speed of the intake motor in rotations per second
     *
     * @return double, the speed of the intake motor in rotations per second
     */
    public double getIntakeSpeed() {
        return intakeMotor.getRotorVelocity().getValue();
    }

    /**
     * Sets the speed of the intake motor
     *
     * @param speed the speed of the intake motor in cm/s
     */
    public void setIntakeSpeed(double speed) {
        targetIntakeSpeed = speed / (2 * Math.PI * Constants.Intake.intakeRollerRadius) / 43;
        //intakeMotor.setControl(velocityVoltage.withVelocity(targetIntakeSpeed));
    }

    @Override
    public void periodic() {
        isBeamBroken = beamDebouncer.calculate(!beamBreak.get());
        isSwivelEnabled = SmartDashboard.getBoolean("Swivel Enabled", true);
        if (isSwivelEnabled) {
            double deltaTime = (System.currentTimeMillis() - swivelMovementStartTime) / 1000;
            double currentAngle = getSwivelPosition();

            TrapezoidProfile.State setPoint = swivelTrapezoidProfile.calculate(deltaTime, new TrapezoidProfile.State(swivelMovementStartAngle, 0), new TrapezoidProfile.State(swivelSetpoint, 0));

            double maxVoltage = RobotController.getBatteryVoltage() * 0.95; // TODO: maybe replace with the PDH voltage?

            double feedforwardOutput = swivelFeedforwardController.calculate(Math.toRadians(setPoint.position), Math.toRadians(setPoint.velocity), 0);
            double swivelPidOutput = swivelPIDController.calculate(currentAngle, setPoint.position);

            double combinedOutput = feedforwardOutput + swivelPidOutput;

            combinedOutput = Math.min(combinedOutput, maxVoltage);
            combinedOutput = Math.max(combinedOutput, -maxVoltage);

            // TelemetryUpdater.setTelemetryValue("Swivel Output Voltage", combinedOutput);
            // TelemetryUpdater.setTelemetryValue("PID output voltage", swivelPidOutput);
            // TelemetryUpdater.setTelemetryValue("setpoint trapezoidal pos", setPoint.position);
            // TelemetryUpdater.setTelemetryValue("setpoint trapezoidal vel", setPoint.velocity);
            // TelemetryUpdater.setTelemetryValue("Trapezoidal ETA", trapezoidProfile.totalTime());

            leftSwivelMotor.setVoltage(combinedOutput);
            rightSwivelMotor.setVoltage(combinedOutput);
            TelemetryUpdater.setTelemetryValue("Intake Swivel Position", currentAngle);
            TelemetryUpdater.setTelemetryValue("raw intake swivel value", angleEncoder.getPosition());
        } else {
            stopAll();
        }

        //double intakePidOutput = intakePIDController.calculate(getIntakeSpeed(), targetIntakeSpeed);

        intakeMotor.set(targetIntakeSpeed);


        // TelemetryUpdater.setTelemetryValue("Total Intake Current Draw", getCurrentDraw());
        TelemetryUpdater.setTelemetryValue("setpoint swivel", swivelSetpoint);

        TelemetryUpdater.setTelemetryValue("Intake Speed", getIntakeSpeed());
        TelemetryUpdater.setTelemetryValue("Intake Desired Speed", targetIntakeSpeed);
        // TelemetryUpdater.setTelemetryValue("Intake Speed", getIntakeSpeed());

        //TelemetryUpdater.setTelemetryValue("Intake Desired Position", swivelSetpoint);
    }

    /**
     * Returns if the beam is broken
     *
     * @return boolean, true if the beam is broken
     */
    public boolean beamBroken() {
        return isBeamBroken;
    }
}
