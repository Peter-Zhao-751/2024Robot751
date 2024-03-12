package frc.robot.utility;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS5Controller.Axis;
import edu.wpi.first.wpilibj.PS5Controller.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class PS5Controller {
    /*
     * precise control: left bumper
     * aimbot: right bumper
     * shoot: right trigger
     * intake: left trigger
     *
     * zero pigeon (for field based control and stuff): triangle
     * zero modules (should be automatic, but in case something goes wrong in the game): circle
     * cross wheels (maybe auto do this when shooting?): square
     *
     * https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj/PS5Controller.Button.html
     */

    /* Axes */
    public static final int translationAxis = Axis.kLeftY.value;
    public static final int strafeAxis = Axis.kLeftX.value;
    public static final int rotationAxis = Axis.kRightX.value;

    /* Controllers */
    public final Joystick joystick;

    /* Triggers */

    public final JoystickButton leftTrigger; // INTAKE
    public final JoystickButton rightTrigger; // SHOOT

    /* Trigger Buttons */

    public final JoystickButton leftTriggerButton; // unassigned
    public final JoystickButton rightTriggerButton; // unassigned

    /* Bumpers */

    public final JoystickButton leftBumper; // AIM BOT
    public final JoystickButton rightBumper; // PRECISE CONTROL

    /* Buttons */

    public final JoystickButton triangleButton;
    public final JoystickButton circleButton;
    public final JoystickButton squareButton;
    public final JoystickButton crossButton;

    /* D-Pad */
    public final DirectionPad dPad;

    /* Other Buttons */
    public final JoystickButton optionsButton; // ZERO PIGEON
    public final JoystickButton playstationButton; // ZERO MODULES

    public PS5Controller() {
        this(0);
    }

    public static class DirectionPad {
        public Joystick joystick;
        public static final int dPadCorrection = 30;
        public final Trigger up;
        public final Trigger down;
        public final Trigger left;
        public final Trigger right;
        private DirectionPad(Joystick joystick) {
            this.joystick = joystick;
            up = new Trigger(this::isUp);
            down = new Trigger(this::isDown);
            left = new Trigger(this::isLeft);
            right = new Trigger(this::isRight);
        }
        private int get() {
            //TelemetryUpdater.setTelemetryValue("D-Pad", joystick.getPOV());
            return joystick.getPOV();
        }
        private boolean isUp() {
            return withinCorrection(this.get(), 0);
        }
        private boolean isDown() {
            return withinCorrection(this.get(), 180);
        }
        private boolean isLeft() {
            return withinCorrection(this.get(), 270);
        }
        private boolean isRight() {
            return withinCorrection(this.get(), 90);
        }

        //whileUp, whileDown, whileLeft, whileRight (repeating commands like Joystick.whileTrue())
        
        private boolean withinCorrection(final int currentDirection, final int desiredDirection) {
            return currentDirection >= desiredDirection - dPadCorrection && currentDirection <= desiredDirection + dPadCorrection;
        }
    }

    
    public PS5Controller(int port) {
        joystick = new Joystick(port);

        leftTrigger = new JoystickButton(joystick, Button.kL2.value);
        rightTrigger = new JoystickButton(joystick, Button.kR2.value);

        leftTriggerButton = new JoystickButton(joystick, Button.kL3.value);
        rightTriggerButton = new JoystickButton(joystick, Button.kR3.value);

        leftBumper = new JoystickButton(joystick, Button.kL1.value);
        rightBumper = new JoystickButton(joystick, Button.kR1.value);

        triangleButton = new JoystickButton(joystick, Button.kTriangle.value);
        circleButton = new JoystickButton(joystick, Button.kCircle.value);
        squareButton = new JoystickButton(joystick, Button.kSquare.value);
        crossButton = new JoystickButton(joystick, Button.kCross.value);

        dPad = new DirectionPad(joystick);

        optionsButton = new JoystickButton(joystick, Button.kOptions.value);
        playstationButton = new JoystickButton(joystick, Button.kPS.value);
    }
}
