package org.firstinspires.ftc.teamcode.Main.Teleop;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Main.Auto.RedThreeBall;
import org.firstinspires.ftc.teamcode.Main.Commands.ChangeDTSpeed;
import org.firstinspires.ftc.teamcode.Main.Commands.Groups.FirePayload;
import org.firstinspires.ftc.teamcode.Main.Commands.Indexer.MoveIndexerArmInCommand;
import org.firstinspires.ftc.teamcode.Main.Commands.Indexer.MoveIndexerArmOutCommand;
import org.firstinspires.ftc.teamcode.Main.Commands.Intake.ForwardIntakeCommand;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Indexer;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Main.Subsystems.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Turret;

@TeleOp(name="Red Solo V2", group="!Solo")
public class RedSolo extends OpMode {
    //Buttons!
    Button aButton;
    Button bButton;
    Button yButton;
    Button xButton;
    Button dpadLeft;
    Button dpadRight;
    Button leftButton;
    Trigger leftTrigger;

    Pose startPose = RedThreeBall.endPose;
    MecanumDrivetrain mecanumDrivetrain;
    Intake intake;
    Indexer indexer;
    Shooter shooter;
    Turret turret;
    FirePayload firePayload;
    TelemetryManager panelsTelemetry;

    @Override
    public void init() {
        GamepadEx pilotGamepad = new GamepadEx(gamepad1);
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        //Button assignment
        aButton = pilotGamepad.getGamepadButton(GamepadKeys.Button.A);
        bButton = pilotGamepad.getGamepadButton(GamepadKeys.Button.B);
        xButton = pilotGamepad.getGamepadButton(GamepadKeys.Button.X);
        yButton = pilotGamepad.getGamepadButton(GamepadKeys.Button.Y);
        dpadLeft = pilotGamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT);
        dpadRight = pilotGamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT);

        mecanumDrivetrain = new MecanumDrivetrain(hardwareMap,startPose,false);
        intake = new Intake(hardwareMap);
        indexer = new Indexer(hardwareMap);
        shooter = new Shooter(hardwareMap);
        turret = new Turret(hardwareMap);

        firePayload = new FirePayload(intake,indexer,shooter,turret,mecanumDrivetrain);
    }

    @Override
    public void start() {
        indexer.setup();
        shooter.setup();
    }

    @Override
    public void loop() {
        aButton.whenPressed(new ChangeDTSpeed(mecanumDrivetrain));
        yButton.whenPressed(firePayload);
        dpadLeft.whenPressed(new MoveIndexerArmOutCommand(indexer));
        dpadRight.whenPressed(new MoveIndexerArmInCommand(indexer));

        leftTrigger.toggleWhenActive(new ForwardIntakeCommand(intake));
    }

    public void pushTelemetry() {

    }
}
