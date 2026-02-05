package org.firstinspires.ftc.teamcode.Main.Teleop.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;

import org.firstinspires.ftc.teamcode.Main.Commands.SpinupFlywheelCommand;
import org.firstinspires.ftc.teamcode.Main.Helpers.DeviceRegistry;
import org.slf4j.helpers.FormattingTuple;

import java.util.ArrayList;

@TeleOp(name = "Servo Turret Test", group = "Testing/Sensors")
public class ServoTurretTest extends OpMode {
    GamepadEx gamepadEx;
    Button aButton;
    Button bButton;
    Button dpadUp;
    Button dpadLeft;
    Button dpadRight;
    CRServoEx firstServo;
    CRServoEx secondServo;
    CRServoEx thirdServo;
    InstantCommand StopServosCommand = new InstantCommand(() -> {
        firstServo.stop(); secondServo.stop(); thirdServo.stop();
    });

    ArrayList<CRServoEx> servoList = new ArrayList<>();
    int stepIndex = 0;

    //TODO test as a servo group
    @Override
    public void init() {
        firstServo = new CRServoEx(hardwareMap, DeviceRegistry.TURRET_SERVO_ONE.str());
        secondServo = new CRServoEx(hardwareMap, DeviceRegistry.TURRET_SERVO_TWO.str());
        thirdServo = new CRServoEx(hardwareMap, DeviceRegistry.TURRET_SERVO_THREE.str());
        telemetry.setMsTransmissionInterval(5);

        firstServo.setInverted(true);
        secondServo.setInverted(false);
        thirdServo.setInverted(true);

        servoList.add(firstServo);
        servoList.add(secondServo);
        servoList.add(thirdServo);

        gamepadEx = new GamepadEx(gamepad1);

        aButton = gamepadEx.getGamepadButton(GamepadKeys.Button.A);
        bButton = gamepadEx.getGamepadButton(GamepadKeys.Button.B);
        dpadUp = gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_UP);
        dpadLeft = gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_LEFT);
        dpadRight = gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT);
    }

    @Override
    public void loop() {
        /*
        A -> all servos forward
        B -> all servos backwards
        dpadUp -> Change current servo
        dpadRight -> Move current servo forward
        dpadLeft -> Move current servo backwards
         */

        CRServoEx currentServo = servoList.get(stepIndex);

        //Run all servos forward
        aButton.whenPressed(new InstantCommand(() -> {
            firstServo.set(1); secondServo.set(1); thirdServo.set(1);
        })).whenReleased(StopServosCommand);

        //Run all servos in reverse
        bButton.whenPressed(new InstantCommand(() -> {
            firstServo.set(-1); secondServo.set(-1); thirdServo.set(-1);
        })).whenReleased(StopServosCommand);

        dpadUp.whenPressed(new InstantCommand(() -> {
            stepIndex = (stepIndex + 1) % servoList.size();
        }));

        dpadRight.whenPressed(new InstantCommand(() -> {
            currentServo.set(1);
        })).whenReleased(StopServosCommand);

        dpadLeft.whenPressed(new InstantCommand(() -> {
            currentServo.set(-1);
        })).whenReleased(StopServosCommand);

        telemetry.addLine("Servo #: " + (stepIndex));
        telemetry.addLine("Servo Inverted?: " + currentServo.getInverted());
        telemetry.addLine("Servo Power: " + currentServo.getServo().getPower());

        CommandScheduler.getInstance().run();
        telemetry.update();
    }
}
