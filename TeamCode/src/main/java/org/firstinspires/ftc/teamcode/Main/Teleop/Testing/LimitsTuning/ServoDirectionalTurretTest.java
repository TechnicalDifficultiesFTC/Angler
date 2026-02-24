package org.firstinspires.ftc.teamcode.Main.Teleop.Testing.LimitsTuning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;

import org.firstinspires.ftc.teamcode.Main.Helpers.Config;
import org.firstinspires.ftc.teamcode.Main.Helpers.DeviceRegistry;

import java.util.ArrayList;

@TeleOp(name = "Servo Directional Turret Test", group = "Turret Servo Stuff")
public class ServoDirectionalTurretTest extends OpMode {
    GamepadEx gamepadEx;
    Button aButton;
    Button bButton;
    Button dpadUp;
    Button dpadLeft;
    Button dpadRight;
    CRServoEx firstServo;
    CRServoEx secondServo;
    CRServoEx thirdServo;
    CRServoEx[] CRServoArr;
    InstantCommand StopServosCommand = new InstantCommand(() -> {
        firstServo.stop(); secondServo.stop(); thirdServo.stop();
    });

    int stepIndex = 0;

    //TODO test as a servo group
    @Override
    public void init() {
        firstServo = new CRServoEx(hardwareMap, DeviceRegistry.TURRET_SERVO_FRONT.str());
        secondServo = new CRServoEx(hardwareMap, DeviceRegistry.TURRET_SERVO_CENTER.str());
        thirdServo = new CRServoEx(hardwareMap, DeviceRegistry.TURRET_SERVO_REAR.str());
        telemetry.setMsTransmissionInterval(5);

        firstServo.setInverted(Config.TurretConstants.TurretServoDirections.frontServoInverted);
        secondServo.setInverted(Config.TurretConstants.TurretServoDirections.centerServoInverted);
        thirdServo.setInverted(Config.TurretConstants.TurretServoDirections.rearServoInverted);

        CRServoArr = new CRServoEx[3];
        CRServoArr[0] = firstServo;
        CRServoArr[1] = secondServo;
        CRServoArr[2] = thirdServo;

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

        //Run all servos forward
        aButton.whenPressed(new InstantCommand(() -> {
            firstServo.set(1); secondServo.set(1); thirdServo.set(1);
        })).whenReleased(StopServosCommand);

        //Run all servos in reverse
        bButton.whenPressed(new InstantCommand(() -> {
            firstServo.set(-1); secondServo.set(-1); thirdServo.set(-1);
        })).whenReleased(StopServosCommand);

        dpadUp.whenPressed(new InstantCommand(() -> {
            stepIndex = (stepIndex + 1) % CRServoArr.length;
        }));

        dpadRight.whenPressed(new InstantCommand(() -> {
            CRServoEx currentServo = CRServoArr[stepIndex];
            currentServo.set(1);
        })).whenReleased(StopServosCommand);

        dpadLeft.whenPressed(new InstantCommand(() -> {
            CRServoEx currentServo = CRServoArr[stepIndex];
            currentServo.set(-1);
        })).whenReleased(StopServosCommand);

        CRServoEx currentServo = CRServoArr[stepIndex];
        telemetry.addLine("Servo #: " + (stepIndex+1));
        telemetry.addLine("Servo Inverted?: " + currentServo.getInverted());
        telemetry.addLine("Servo Power: " + currentServo.getServo().getPower());

        CommandScheduler.getInstance().run();
        telemetry.update();
    }
}
