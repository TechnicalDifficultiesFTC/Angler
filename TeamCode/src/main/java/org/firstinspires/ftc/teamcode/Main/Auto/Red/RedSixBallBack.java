package org.firstinspires.ftc.teamcode.Main.Auto.Red;

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
import com.seattlesolvers.solverslib.util.Timing;

import org.firstinspires.ftc.teamcode.Main.Helpers.Config;
import org.firstinspires.ftc.teamcode.Main.Helpers.Drawing;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Indexer;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Main.Subsystems.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "Red 6 Ball Back", group = "!Autonomous")
@Configurable // Panels
public class RedSixBallBack extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState = 0; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private Timer pathTimer, actionTimer, opmodeTimer;
    private final Pose startPose = new Pose(87.486, 8.695, Math.toRadians(90));
    private Shooter shooter;
    private Indexer indexer;
    private Intake intake;
    private boolean shootCommandIncomplete;
    private int shotsFired = 0;
    boolean isBlue = false;
    MecanumDrivetrain mecanumDrivetrain;
    public static Pose endPose;
    public Timing.Timer firstTimer;

    @Override
    public void init() {

        mecanumDrivetrain = new MecanumDrivetrain(hardwareMap, startPose, isBlue);
        shooter = new Shooter(hardwareMap);
        indexer = new Indexer(hardwareMap);
        intake = new Intake(hardwareMap);

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.setMaxPower(.5);

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
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

        follower.update(); // Update Pedro Pathing
        autonomousPathUpdate(); // Update autonomous state machine
        Drawing.drawDebug(follower);
        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.debug("Shooter command complete?: " + !shootCommandIncomplete);
        panelsTelemetry.debug("Shots Fired: " + shotsFired);
        panelsTelemetry.debug("Shooter rec power?: " + shooter.getSpeedILUTValue(mecanumDrivetrain.getEstimatedDistanceToGoal()));
        panelsTelemetry.update(telemetry);
    }


    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(86.559, 10.084),

                                    new Pose(87.023, 13.543)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(60))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(87.023, 13.543),

                                    new Pose(90.058, 49.543)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(90.058, 49.543),

                                    new Pose(128.601, 49.875)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(128.601, 49.875),

                                    new Pose(89.148, 35.826)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(60))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(89.148, 35.826),

                                    new Pose(86.759, 15.064)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(60))

                    .build();
        }
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    public void autonomousPathUpdate() {
        // Add your state machine Here
        // Access paths with paths.pathName
        // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
        switch (pathState) {
            case 0:
                follower.followPath(paths.Path1);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    // Path2 just finished — start timer and hold position
                    firstTimer = new Timing.Timer(2000, TimeUnit.MILLISECONDS);
                    firstTimer.start();
                    follower.holdPoint(new Pose(87.023, 13.543,Math.toRadians(60)));
                    setPathState(2);
                }
                break;
            case 2:
                if(firstTimer.done()) {
                    follower.followPath(paths.Path2);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    follower.followPath(paths.Path3);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    follower.followPath(paths.Path4);
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy()) {
                    follower.followPath(paths.Path5);
                    setPathState(6);
                }
                break;
        }

    }
}
