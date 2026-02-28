package org.firstinspires.ftc.teamcode.Main.Teleop.Testing.HyperTests;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Main.Commands.Drivetrain.FlipDrivetrainLPM;
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

@TeleOp(name = "OTM PID's", group = "PID")
public class OnTheMovePIDTuning extends OpMode {
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
    Pose startingPose = new Pose(120,125,Math.toRadians(36));
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

    int stepIndex;
    double[] stepSizes = {10.0, 1.0, 0.1, 0.001, 0.0001};

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
        mecanumDrivetrain = new MecanumDrivetrain(hardwareMap, startingPose,false);

        fireOnce = new FireOnce(intake,indexer,shooter,turret,mecanumDrivetrain);
        fireSalvo = new FirePayload(intake,indexer,shooter,turret,mecanumDrivetrain);
        aimTurretCommand = new AimTurretCommand(turret, mecanumDrivetrain, shooter, telemetry);
    }

    @Override
    public void start() {
        shooter.setup();
        indexer.setup();
        //This command will never terminate
        commandSchedulerInstance.schedule(new UpdateIndexerState(indexer,intake,shooter,panelsTelem));
        commandSchedulerInstance.schedule(aimTurretCommand);
    }
    boolean tuneLong = false;
    @Override
    public void loop() {
        mecanumDrivetrain.processInputRC(gamepad1);
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

        bButton.whenPressed(fireSalvo);
        yButton.whenPressed(fireOnce);
        aButton.whenPressed(new FlipDrivetrainLPM(mecanumDrivetrain));

        double normalized = turret.normalizeDegrees(aimTurretCommand.rcTheta);
        telemetry.addLine("Brake State: " + mecanumDrivetrain.backLeftMotor.getZeroPowerBehavior().toString());
        telemetry.addLine("Short PID: " + turret.usingShortRange);
        telemetry.addLine("Error: " + Utils.ras((aimTurretCommand.rcTheta - turret.getCurrentPositionAsDegrees())));
        telemetry.addLine("RC Theta: " + Utils.ras(aimTurretCommand.rcTheta));
        telemetry.addLine("");

        telemetry.addLine("Normalize Degrees: " + Utils.ras(normalized));
        telemetry.addLine("PIDF: " + Utils.ras(turret.runPIDFControllers(normalized)));

        telemetry.addLine("");
        telemetry.addLine("Indexer motor power: " + indexer.indexerMotor.getPower());
        telemetry.addLine("Shooter ready?: " + shooter.isFlywheelReady());
        telemetry.addLine("LPM: " + mecanumDrivetrain.isLowPowerMode());
        telemetry.addLine("Shoot command?: " + commandSchedulerInstance.isScheduled(fireOnce));
        telemetry.addLine();
        //telemetry.addLine("pS: " + )

        double distance = mecanumDrivetrain.getEstimatedDistanceToGoal();
        telemetry.addLine("Distance: " + distance);
        telemetry.addLine("Est Speed/Hood: " + shooter.getHoodILUTValue(distance) + "/" + shooter.getSpeedILUTValue(distance));

        if (gamepad2.dpadUpWasPressed()) {
            if (tuneLong) {
                pL += stepSizes[stepIndex];
            } else {
                pS += stepSizes[stepIndex];
            }
        }

        if (gamepad2.dpadDownWasPressed()) {
            if (tuneLong) {
                pL -= stepSizes[stepIndex];
            } else {
                pS += stepSizes[stepIndex];
            }
        }

        if (gamepad2.xWasPressed()) {
            if (tuneLong) {
                dL -= stepSizes[stepIndex];
            } else {
                dS -= stepSizes[stepIndex];
            }
        }

        if (gamepad2.aWasPressed()) {
            tuneLong = !tuneLong;
        }

        if (gamepad2.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad2.yWasPressed()) {
            if (tuneLong) {
                dL += stepSizes[stepIndex];
            } else {
                dS += stepSizes[stepIndex];
            }
        }

        if (gamepad2.dpadLeftWasPressed()) {
            if (tuneLong) {
                fL -= stepSizes[stepIndex];
            } else {
                fS -= stepSizes[stepIndex];
            }
        }

        if (gamepad2.dpadRightWasPressed()) {
            if (tuneLong) {
                fL += stepSizes[stepIndex];
            } else {
                fS += stepSizes[stepIndex];
            }
        }

        telemetry.addLine("Step: " + stepSizes[stepIndex]);

        telemetry.addLine();
        telemetry.addLine("Long P: " + Utils.ras(pL));
        telemetry.addLine("Long D: " + Utils.ras(dL));
        telemetry.addLine("Long F: " + Utils.ras(fL));

        telemetry.addLine();
        telemetry.addLine("Short P: " + Utils.ras(pS));
        telemetry.addLine("Short D: " + Utils.ras(dS));
        telemetry.addLine("Short F: " + Utils.ras(fS));

        telemetry.addLine();
        telemetry.addLine("LONG: " + turret.getLongPIDFCoefficients());
        telemetry.addLine("SHORT:" + turret.getShortPIDFCoefficients());

        turret.setLongPIDFCoeffecients(new PIDFCoefficients(pL, iL, dL, fL));
        turret.setShortPIDFCoeffecients(new PIDFCoefficients(pS, iS, dS, fS));

        panelsTelem.update(telemetry);
        commandSchedulerInstance.run();
    }

    public boolean cmdSch(Command command) {
        return commandSchedulerInstance.isScheduled(command);
    }
}
