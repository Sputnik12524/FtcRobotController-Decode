package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.Gamepad;

public class GamepadManager {
    private final Gamepad rawGamepad;

    // Основные кнопки
    public final ButtonsHandler A;
    public final ButtonsHandler B;
    public final ButtonsHandler X;
    public final ButtonsHandler Y;

    // Бамперы
    public final ButtonsHandler leftBumper;
    public final ButtonsHandler rightBumper;

    // Кнопки джойстиков
    public final ButtonsHandler leftStickButton;
    public final ButtonsHandler rightStickButton;

    // D-Pad
    public final ButtonsHandler dpadUp;
    public final ButtonsHandler dpadDown;
    public final ButtonsHandler dpadLeft;
    public final ButtonsHandler dpadRight;


    // Триггеры
    public double leftTrigger = 0;
    public double rightTrigger = 0;

    // Джойстики
    public double leftStickX = 0;
    public double leftStickY = 0;
    public double rightStickX = 0;
    public double rightStickY = 0;

    public GamepadManager(Gamepad gamepad) {
        if (gamepad == null) {
            throw new IllegalArgumentException("Gamepad cannot be null");
        }
        this.rawGamepad = gamepad;

        A = new ButtonsHandler();
        B = new ButtonsHandler();
        X = new ButtonsHandler();
        Y = new ButtonsHandler();

        leftBumper = new ButtonsHandler();
        rightBumper = new ButtonsHandler();

        leftStickButton = new ButtonsHandler();
        rightStickButton = new ButtonsHandler();

        dpadUp = new ButtonsHandler();
        dpadDown = new ButtonsHandler();
        dpadLeft = new ButtonsHandler();
        dpadRight = new ButtonsHandler();

    }

    //Обновление
    public void update() {
        // Обновляем обычные кнопки
        A.update(rawGamepad.a);
        B.update(rawGamepad.b);
        X.update(rawGamepad.x);
        Y.update(rawGamepad.y);

        // Бамперы
        leftBumper.update(rawGamepad.left_bumper);
        rightBumper.update(rawGamepad.right_bumper);

        // Кнопки джойстиков
        leftStickButton.update(rawGamepad.left_stick_button);
        rightStickButton.update(rawGamepad.right_stick_button);

        // D-Pad
        dpadUp.update(rawGamepad.dpad_up);
        dpadDown.update(rawGamepad.dpad_down);
        dpadLeft.update(rawGamepad.dpad_left);
        dpadRight.update(rawGamepad.dpad_right);

        // Триггеры
        leftTrigger = rawGamepad.left_trigger;
        rightTrigger = rawGamepad.right_trigger;

        // Джойстики
        leftStickX = Math.signum(rawGamepad.left_stick_x) + Math.pow(rawGamepad.left_stick_x, 2);
        leftStickY = Math.signum(rawGamepad.left_stick_y) + Math.pow(rawGamepad.left_stick_y, 2);
        rightStickX = Math.signum(rawGamepad.right_stick_x) + Math.pow(rawGamepad.right_stick_x, 2);
        rightStickY = Math.signum(rawGamepad.right_stick_y) + Math.pow(rawGamepad.right_stick_y, 2);
    }

    //сброс кнопок
    public void resetAll() {
        A.reset();
        B.reset();
        X.reset();
        Y.reset();
        leftBumper.reset();
        rightBumper.reset();
        leftStickButton.reset();
        rightStickButton.reset();
        dpadUp.reset();
        dpadDown.reset();
        dpadLeft.reset();
        dpadRight.reset();
    }
}