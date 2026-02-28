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

import org.firstinspires.ftc.teamcode.Main.Helpers.Config;
import org.firstinspires.ftc.teamcode.Main.Helpers.Drawing;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Indexer;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Main.Subsystems.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Red 6 Ball Front", group = "!Autonomous")
@Configurable // Panels
public class RedSixBallFront extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState = 0; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private Timer pathTimer, actionTimer, opmodeTimer;
    private static final Pose startPose = new Pose(123.010, 121.990, Math.toRadians(36));
    private Shooter shooter;
    private Indexer indexer;
    private Intake intake;
    private boolean shootCommandIncomplete;
    private int shotsFired = 0;
    boolean isBlue = false;
    MecanumDrivetrain mecanumDrivetrain;
    public static Pose endPose;

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
        follower.setMaxPower(.75);

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

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(123.010, 121.990),

                                    new Pose(87.486, 86.469)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(36))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(87.486, 86.469),

                                    new Pose(121.199, 84.563)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(121.199, 84.563),

                                    new Pose(87.772, 86.531)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(1), Math.toRadians(36))
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
                    follower.followPath(paths.Path2);
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    follower.followPath(paths.Path3);
                }
        }

    }
}
    