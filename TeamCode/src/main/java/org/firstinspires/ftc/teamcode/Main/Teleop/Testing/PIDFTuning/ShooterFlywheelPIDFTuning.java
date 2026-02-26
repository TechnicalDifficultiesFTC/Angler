package org.firstinspires.ftc.teamcode.Main.Teleop.Testing.PIDFTuning;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Main.Commands.Indexer.MoveIndexerArmInCommand;
import org.firstinspires.ftc.teamcode.Main.Commands.Indexer.MoveIndexerArmOutCommand;
import org.firstinspires.ftc.teamcode.Main.Commands.Indexer.UpdateIndexerState;
import org.firstinspires.ftc.teamcode.Main.Commands.Intake.ForwardIntakeCommand;
import org.firstinspires.ftc.teamcode.Main.Commands.Intake.ReverseIntakeCommand;
import org.firstinspires.ftc.teamcode.Main.Commands.Intake.StopIntakeCommand;
import org.firstinspires.ftc.teamcode.Main.Helpers.Config;
import org.firstinspires.ftc.teamcode.Main.Helpers.Utils;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Indexer;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Shooter;
@Configurable
@TeleOp(name = "Turret Flywheel PIDF Tuning", group = "PID")
public class ShooterFlywheelPIDFTuning extends OpMode {
    double curTargetVelocityAsPercentage = 0;
    double error = 0;
    double highVelocityAsPercent = 75;
    double lowVelocityAsPercent = 20;
    int stepIndex;
    double[] stepSizes = {10.0, 1.0, 0.1, 0.001, 0.0001};
    double P = Config.ShooterConstants.FlywheelPIDF.p;
    double I = Config.ShooterConstants.FlywheelPIDF.i;
    double D = Config.ShooterConstants.FlywheelPIDF.d;
    double F = Config.ShooterConstants.FlywheelPIDF.f;
    Shooter shooter;
    String MOTM;
    TelemetryManager panelsTelemetry;
    Intake intake;
    Indexer indexer;
    GamepadEx gamepadEx;
    Button aButton;
    Button bButton;
    Button yButton;
    Button dpadUp;
    Button dpadLeft;
    Button dpadRight;
    Trigger leftTrigger;
    Trigger rightTrigger;

    @Override
    public void init() {
        gamepadEx = new GamepadEx(gamepad1);

        aButton = gamepadEx.getGamepadButton(GamepadKeys.Button.A);
        bButton = gamepadEx.getGamepadButton(GamepadKeys.Button.B);
        yButton = gamepadEx.getGamepadButton(GamepadKeys.Button.Y);
        dpadUp = gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_UP);
        dpadLeft = gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_LEFT);
        dpadRight = gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT);

        leftTrigger = new Trigger(() -> gamepadEx.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) >
                Config.ControllerConstants.TRIGGER_THRESHOLD);
        rightTrigger = new Trigger(() -> gamepadEx.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) >
                Config.ControllerConstants.TRIGGER_THRESHOLD);

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        MOTM = Utils.generateMOTM();
        shooter = new Shooter(hardwareMap);
        indexer = new Indexer(hardwareMap);
        intake = new Intake(hardwareMap);
    }

    public void start() {
        shooter.setup();
        indexer.setup();

        CommandScheduler.getInstance().schedule(new UpdateIndexerState(indexer,intake,shooter));
    }

    @Override
    public void loop() {
        //Switch between high velocity and low velocity

        //Intake indexer controls
        leftTrigger
                .whileActiveOnce(new ForwardIntakeCommand(intake))
                .whenInactive(new StopIntakeCommand(intake));

        rightTrigger
                .whileActiveOnce(new ReverseIntakeCommand(intake))
                .whenInactive(new StopIntakeCommand(intake));

        dpadLeft.whenPressed(new MoveIndexerArmOutCommand(indexer));
        dpadRight.whenPressed(new MoveIndexerArmInCommand(indexer));

        if (gamepad1.yWasPressed()) {
            if (curTargetVelocityAsPercentage == highVelocityAsPercent) {
                curTargetVelocityAsPercentage = lowVelocityAsPercent;

            } else {
                curTargetVelocityAsPercentage = highVelocityAsPercent; }
        }

        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad1.dpadLeftWasPressed()) {
            F -= stepSizes[stepIndex];
        }

        if (gamepad1.dpadRightWasPressed()) {
            F += stepSizes[stepIndex];
        }

        if (gamepad1.dpadUpWasPressed()) {
            P += stepSizes[stepIndex];
        }

        if (gamepad1.dpadDownWasPressed()) {
            P -= stepSizes[stepIndex];
        }

        if (gamepad1.squareWasPressed()) {
            curTargetVelocityAsPercentage = 0;
        }

        shooter.setVelocityPIDF(new PIDFCoefficients(P, 0, 0, F));
        shooter.setFlywheelTargetVelocityAsPercentage(curTargetVelocityAsPercentage);

        double curVelocityAsPercentage = shooter.getFlywheelCurrentVelocityAsPercentage();
        error = curTargetVelocityAsPercentage - shooter.getFlywheelCurrentVelocityAsPercentage();

        panelsTelemetry.addLine("MOTM: " + MOTM);
        panelsTelemetry.addLine("");
        panelsTelemetry.addLine("Step Value: " + stepSizes[stepIndex]);
        panelsTelemetry.addLine("");

        panelsTelemetry.addData("Error ", Utils.ras(Math.abs(error)));
        panelsTelemetry.addData("Setpoint ", Utils.ras(curTargetVelocityAsPercentage));
        panelsTelemetry.addData("Velocity ", Utils.ras(curVelocityAsPercentage));

        panelsTelemetry.addLine("");
        panelsTelemetry.addLine("Ready?: " + shooter.isFlywheelReady());

        panelsTelemetry.addData("P", shooter.flywheelMotorLeft.getPIDFCoefficients
                (DcMotor.RunMode.RUN_USING_ENCODER).p);
        panelsTelemetry.addData("F", shooter.flywheelMotorLeft.getPIDFCoefficients
                (DcMotor.RunMode.RUN_USING_ENCODER).f);

        panelsTelemetry.update(telemetry);
        CommandScheduler.getInstance().run();
    }
}
