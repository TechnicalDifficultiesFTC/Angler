package org.firstinspires.ftc.teamcode.Main.Teleop;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Main.Auto.Blue.BlueSixBallBack;
import org.firstinspires.ftc.teamcode.Main.Auto.Blue.BlueSixBallFront;
import org.firstinspires.ftc.teamcode.Main.Auto.Red.RedSixBallBack;
import org.firstinspires.ftc.teamcode.Main.Auto.Red.RedSixBallFront;
import org.firstinspires.ftc.teamcode.Main.Commands.Groups.FireOnce;
import org.firstinspires.ftc.teamcode.Main.Commands.Groups.FirePayload;
import org.firstinspires.ftc.teamcode.Main.Commands.Indexer.MoveIndexerArmInCommand;
import org.firstinspires.ftc.teamcode.Main.Commands.Indexer.MoveIndexerArmOutCommand;
import org.firstinspires.ftc.teamcode.Main.Commands.Indexer.UpdateIndexerState;
import org.firstinspires.ftc.teamcode.Main.Commands.Turret.AimTurretCommand;
import org.firstinspires.ftc.teamcode.Main.Helpers.Config;
import org.firstinspires.ftc.teamcode.Main.Helpers.Utils;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Indexer;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Main.Subsystems.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Turret;

@TeleOp(name = "Blue TeleOp | MAV SUPPORT", group = "!TELEOP")
public class BlueMav extends OpMode {
    String MOTM;
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
    Button rightBumper;
    Trigger leftTrigger;
    Trigger rightTrigger;
    TelemetryManager panelsTelem;
    Pose startingPose = Config.AutoPoses.defaultPoseRed.mirror();
    FireOnce fireOnce;
    FirePayload fireSalvo;
    AimTurretCommand aimTurretCommand;

    double pL = Config.TurretConstants.TurretPIDFLarge.p;
    double iL = Config.TurretConstants.TurretPIDFLarge.i;
    double dL = Config.TurretConstants.TurretPIDFLarge.d;
    double fL = Config.TurretConstants.TurretPIDFLarge.f;

    //Small
    double pS = Config.TurretConstants.TurretPIDFSmall.p;
    double iS = Config.TurretConstants.TurretPIDFSmall.i;
    double dS = Config.TurretConstants.TurretPIDFSmall.d;
    double fS = Config.TurretConstants.TurretPIDFSmall.f;
    //TODO Check me
    boolean isBlue = true;
    UpdateIndexerState updateIndexerState;

    @Override
    public void init() {
        MOTM = Utils.generateMOTM();

        // Check if an auto was just run and use its end pose
        Pose autoPose = null;

        if (BlueSixBallBack.endPose != null) {
            autoPose = BlueSixBallBack.endPose;
            BlueSixBallBack.endPose = null; // clear it

        } else if (BlueSixBallFront.endPose != null) {
            autoPose = BlueSixBallFront.endPose;
            BlueSixBallFront.endPose = null; // clear it
        }

        if (autoPose == null) {
            autoPose = startingPose;
        }

        gamepadEx = new GamepadEx(gamepad1);
        panelsTelem = PanelsTelemetry.INSTANCE.getTelemetry();

        aButton = gamepadEx.getGamepadButton(GamepadKeys.Button.A);
        bButton = gamepadEx.getGamepadButton(GamepadKeys.Button.B);
        yButton = gamepadEx.getGamepadButton(GamepadKeys.Button.Y);
        dpadUp = gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_UP);
        dpadLeft = gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_LEFT);
        dpadRight = gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT);
        rightBumper = gamepadEx.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER);

        leftTrigger = new Trigger(() -> gamepadEx.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) >
                Config.ControllerConstants.TRIGGER_THRESHOLD);
        rightTrigger = new Trigger(() -> gamepadEx.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) >
                Config.ControllerConstants.TRIGGER_THRESHOLD);

        commandSchedulerInstance = CommandScheduler.getInstance();

        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap,gamepad1);
        indexer = new Indexer(hardwareMap);
        turret = new Turret(hardwareMap);

        mecanumDrivetrain = new MecanumDrivetrain(hardwareMap, autoPose,isBlue);

        fireOnce = new FireOnce(intake,indexer,shooter,turret,mecanumDrivetrain);
        fireSalvo = new FirePayload(intake,indexer,shooter,turret,mecanumDrivetrain);
        aimTurretCommand = new AimTurretCommand(turret, mecanumDrivetrain, shooter, telemetry);
        updateIndexerState = new UpdateIndexerState(indexer,intake,shooter,panelsTelem);

        // When right bumper is pressed, cancel UpdateIndexerState and run reverse indexer
        // When released, stop reverse and reschedule UpdateIndexerState
        rightBumper
                .whileActiveOnce(
                        new InstantCommand(() -> {
                            commandSchedulerInstance.cancel(updateIndexerState);
                            indexer.indexerReverse(); // or whatever your reverse method is called
                        })
                )
                .whenInactive(
                        new InstantCommand(() -> {
                            indexer.indexerStop(); // stop reversing
                            commandSchedulerInstance.schedule(updateIndexerState);
                        })
                );
    }

    @Override
    public void start() {
        shooter.setup();
        indexer.setup();
        //This command will never terminate
        commandSchedulerInstance.schedule(updateIndexerState);
        commandSchedulerInstance.schedule(aimTurretCommand);
    }

    boolean tuneLong = false;
    @Override
    public void loop() {
        //Pilot Controls:
        mecanumDrivetrain.processInputRC(gamepad1);

        dpadLeft.whenPressed(new MoveIndexerArmOutCommand(indexer));
        dpadRight.whenPressed(new MoveIndexerArmInCommand(indexer));

        bButton.whenPressed(fireSalvo);
        yButton.whenPressed(fireOnce);

        if (gamepad1.aWasPressed()) {mecanumDrivetrain.setLowPowerMode(!mecanumDrivetrain.isLowPowerMode());}

        //MAV Controls:

        if (gamepad2.aWasPressed()) {
            CommandScheduler.getInstance().cancel(aimTurretCommand);
            turret.realSetTurretPositionAsDegrees(0);
        }

        if (gamepad2.bWasPressed()) {
            mecanumDrivetrain.getFollower().setPose(startingPose);
        }

        telemetry.addLine(MOTM);
        telemetry.addLine();
        telemetry.addLine("Pose: " + mecanumDrivetrain.getPose().toString());
        telemetry.addLine();
        telemetry.addLine("Error: " + Utils.ras((aimTurretCommand.rcTheta - turret.getCurrentPositionAsDegrees())));
        telemetry.addLine("RC Theta: " + Utils.ras(aimTurretCommand.rcTheta));
        telemetry.addLine("Turret Active?: " + CommandScheduler.getInstance().isScheduled(aimTurretCommand));
        telemetry.addLine();

        double distance = mecanumDrivetrain.getEstimatedDistanceToGoal();
        telemetry.addLine("Distance: " + distance);
        telemetry.addLine("Speed/Hood: " + Utils.ras(shooter.getHoodILUTValue(distance)) + "/" + Utils.ras(shooter.getSpeedILUTValue(distance)));

        turret.setLongPIDFCoeffecients(new PIDFCoefficients(pL, iL, dL, fL));
        turret.setShortPIDFCoeffecients(new PIDFCoefficients(pS, iS, dS, fS));

        panelsTelem.update(telemetry);
        commandSchedulerInstance.run();
    }
}
