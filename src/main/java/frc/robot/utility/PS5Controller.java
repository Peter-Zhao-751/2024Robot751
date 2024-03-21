package frc.robot.utility;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS5Controller.Axis;
import edu.wpi.first.wpilibj.PS5Controller.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class PS5Controller {
    /* Axes */
    public static final int translationAxis = Axis.kLeftY.value;
    public static final int strafeAxis = Axis.kLeftX.value;
    public static final int rotationAxis = Axis.kRightX.value;

    /* Controllers */
    public final Joystick joystick;

    /* Triggers */
    public final JoystickButton leftTrigger;
    public final JoystickButton rightTrigger;

    /* Trigger Buttons */
    public final JoystickButton leftTriggerButton;
    public final JoystickButton rightTriggerButton;

    /* Bumpers */
    public final JoystickButton leftBumper;
    public final JoystickButton rightBumper;

    /* Buttons */
    public final JoystickButton triangleButton;
    public final JoystickButton circleButton;
    public final JoystickButton squareButton;
    public final JoystickButton crossButton;

    /* D-Pad */
    public final DirectionPad dPad;

    /* Other Buttons */
    public final JoystickButton optionsButton;
    public final JoystickButton playstationButton;

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

    public static class DirectionPad {
        public final Joystick joystick;
        public final Trigger up;
        public final Trigger right;
        public final Trigger down;
        public final Trigger left;
        private DirectionPad(Joystick joystick) {
            this.joystick = joystick;
            up = new Trigger(() -> joystick.getPOV() == 0.0);
            right = new Trigger(() -> joystick.getPOV() == 90.0);
            down = new Trigger(() -> joystick.getPOV() == 180.0);
            left = new Trigger(() -> joystick.getPOV() == 270.0);
        }
    }
}
