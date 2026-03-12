package org.firstinspires.ftc.teamcode.Main.Auto.Blue;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.Main.Commands.Groups.FirePayload;
import org.firstinspires.ftc.teamcode.Main.Commands.Indexer.MoveIndexerArmInCommand;
import org.firstinspires.ftc.teamcode.Main.Commands.Indexer.UpdateIndexerState;
import org.firstinspires.ftc.teamcode.Main.Helpers.Config;
import org.firstinspires.ftc.teamcode.Main.Helpers.Drawing;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Indexer;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Main.Subsystems.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Blue 6 Ball Front", group = "!Blue Autonomous")
@Configurable
public class BlueSixBallFront extends OpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState = 0;
    private Paths paths;
    private Timer pathTimer, opmodeTimer;
    private static final Pose startPose = new Pose(123.010, 121.990, Math.toRadians(36)).mirror();
    private Shooter shooter;
    private Indexer indexer;
    private Intake intake;
    private boolean shootCommandIncomplete;
    private int shotsFired = 0;
    boolean isBlue = true;
    MecanumDrivetrain mecanumDrivetrain;
    Turret turret;
    public static Pose endPose;
    FirePayload firstSalvo;
    FirePayload secondSalvo;

    @Override
    public void init() {
        mecanumDrivetrain = new MecanumDrivetrain(hardwareMap, startPose, isBlue);
        shooter = new Shooter(hardwareMap);
        indexer = new Indexer(hardwareMap);
        intake = new Intake(hardwareMap);
        turret = new Turret(hardwareMap);

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.setMaxPower(.75);

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        paths = new Paths(follower);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        firstSalvo = new FirePayload(intake, indexer, shooter, turret, mecanumDrivetrain);
        secondSalvo = new FirePayload(intake, indexer, shooter, turret, mecanumDrivetrain);
    }

    @Override
    public void start() {
        shooter.setup();
        indexer.setup();

        shooter.setFlywheelTargetVelocityAsPercentage(
                Config.ShooterConstants.FLYWHEEL_SPEED_HOVERING_PERCENTAGE
        );

        intake.intakeSpinup();
        indexer.moveArmIn();
    }

    @Override
    public void loop() {
        endPose = follower.getPose();

        follower.update();
        autonomousPathUpdate();
        Drawing.drawDebug(follower);

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.debug("Shooter command complete?: " + !shootCommandIncomplete);
        panelsTelemetry.debug("Shots Fired: " + shotsFired);
        panelsTelemetry.debug("Shooter rec power?: " + shooter.getSpeedILUTValue(mecanumDrivetrain.getEstimatedDistanceToGoal()));
        panelsTelemetry.update(telemetry);

        CommandScheduler.getInstance().run();
    }


    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(123.010, 122.453).mirror(),

                                    new Pose(87.486, 86.469).mirror()
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180-36), Math.toRadians(180-40))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(87.486, 86.469).mirror(),

                                    new Pose(125.830, 84.100).mirror()
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(125.830, 84.100).mirror(),

                                    new Pose(87.772, 86.531).mirror()
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180-1), Math.toRadians(180-40))
                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(87.772, 86.531).mirror(),

                                    new Pose(87.534, 118.283).mirror()
                            )
                    ).setTangentHeadingInterpolation()

                    .build();
        }
    }


    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(paths.Path1);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    CommandScheduler.getInstance().schedule(firstSalvo);
                    setPathState(2);
                }
                break;

            case 2:
                if (!follower.isBusy() && firstSalvo.isFinished()) {
                    CommandScheduler.getInstance().schedule(new MoveIndexerArmInCommand(indexer));
                    follower.followPath(paths.Path2);
                    setPathState(3);
                }
                break;

            case 3:
//                if (!follower.isBusy()) {
//                    follower.followPath(paths.Path3);
//                    setPathState(4);
//                }
                break;

//            case 4:
//                if (!follower.isBusy()) {
//                    CommandScheduler.getInstance().schedule(secondSalvo);
//                    setPathState(5);
//                }
//                break;
//
//            case 5:
//                if (!follower.isBusy() && !CommandScheduler.getInstance().isScheduled(secondSalvo)) {
//                    shooter.setFlywheelTargetVelocityAsPercentage(0);
//                    setPathState(6);
//                }
//                break;
//
//            case 6:
//                // Terminal state — do nothing
//                break;
        }
    }
}