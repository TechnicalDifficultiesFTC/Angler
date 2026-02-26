package org.firstinspires.ftc.teamcode.Main.Teleop.Testing.CompositeTesting;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;

import org.firstinspires.ftc.teamcode.Main.Commands.Drivetrain.FlipDrivetrainLPM;
import org.firstinspires.ftc.teamcode.Main.Commands.Groups.FireOnce;
import org.firstinspires.ftc.teamcode.Main.Commands.Indexer.MoveIndexerArmInCommand;
import org.firstinspires.ftc.teamcode.Main.Commands.Indexer.MoveIndexerArmOutCommand;
import org.firstinspires.ftc.teamcode.Main.Commands.Indexer.UpdateIndexerState;
import org.firstinspires.ftc.teamcode.Main.Commands.Intake.ForwardIntakeCommand;
import org.firstinspires.ftc.teamcode.Main.Commands.Intake.ReverseIntakeCommand;
import org.firstinspires.ftc.teamcode.Main.Commands.Intake.StopIntakeCommand;
import org.firstinspires.ftc.teamcode.Main.Helpers.Config;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Indexer;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Main.Subsystems.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Turret;

@TeleOp(name = "Command Testing", group = "Misc/Test")
public class CommandTesting extends OpMode {
    Shooter shooter;
    Intake intake;
    Indexer indexer;
    Turret turret;
    MecanumDrivetrain mecanumDrivetrain;
    CommandScheduler commandSchedulerInstance;
    GamepadEx gamepadEx;
    Button aButton;
    Button bButton;
    Button yButton;
    Button dpadUp;
    Button dpadLeft;
    Button dpadRight;
    Trigger leftTrigger;
    Trigger rightTrigger;
    TelemetryManager panelsTelem;
    @Override
    public void init() {
        gamepadEx = new GamepadEx(gamepad1);
        panelsTelem = PanelsTelemetry.INSTANCE.getTelemetry();

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

        commandSchedulerInstance = CommandScheduler.getInstance();

        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap,gamepad1);
        indexer = new Indexer(hardwareMap);
        turret = new Turret(hardwareMap);
        mecanumDrivetrain = new MecanumDrivetrain(hardwareMap,new Pose(),true);
    }

    @Override
    public void start() {
        shooter.setup();
        indexer.setup();
        //This command will never terminate
        commandSchedulerInstance.schedule(new UpdateIndexerState(indexer,intake,shooter,panelsTelem));
    }
    @Override
    public void loop() {
        //TODO: Test periodic intake
//        leftTrigger
//                .whileActiveOnce(new ForwardIntakeCommand(intake))
//                .whenInactive(new StopIntakeCommand(intake));
//
//        rightTrigger
//                .whileActiveOnce(new ReverseIntakeCommand(intake))
//                .whenInactive(new StopIntakeCommand(intake));

        dpadLeft.whenPressed(new MoveIndexerArmOutCommand(indexer));
        dpadRight.whenPressed(new MoveIndexerArmInCommand(indexer));

        yButton.whenPressed(new FireOnce(intake,indexer,shooter,turret,mecanumDrivetrain));
        aButton.whenPressed(new FlipDrivetrainLPM(mecanumDrivetrain));

        telemetry.addLine("Ball seen?: " + indexer.ballDetected());
        telemetry.addLine("Arm out?: " + indexer.isArmInTheWay());
        telemetry.addLine("Indexer motor power: " + indexer.indexerMotor.getPower());
        telemetry.addLine("Shooter ready?: " + shooter.isFlywheelReady());
        panelsTelem.update(telemetry);

        commandSchedulerInstance.run();
    }

    public boolean cmdSch(Command command) {
        return commandSchedulerInstance.isScheduled(command);
    }
}
