package frc.robot.utility;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS5Controller.Axis;
import edu.wpi.first.wpilibj.PS5Controller.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

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

    /* Other Buttons */
    public final JoystickButton optionsButton; // ZERO PIGEON
    public final JoystickButton playstationButton; // ZERO MODULES

    public PS5Controller() {
        this(0);
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

        optionsButton = new JoystickButton(joystick, Button.kOptions.value);
        playstationButton = new JoystickButton(joystick, Button.kPS.value);
    }
}
