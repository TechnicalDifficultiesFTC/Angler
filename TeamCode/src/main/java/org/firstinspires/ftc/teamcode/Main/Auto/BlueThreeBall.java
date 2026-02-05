package org.firstinspires.ftc.teamcode.Main.Auto;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.teamcode.Main.Helpers.Config;
import org.firstinspires.ftc.teamcode.Main.Helpers.Utils;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Indexer;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Main.Subsystems.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

@Autonomous(name = "Blue 3 Ball", group = "!Autonomous")
@Configurable // Panels
public class BlueThreeBall extends OpMode {
    public Pose autoEndPose;
    private int shotsFired;
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState = 0; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private Timer pathTimer, actionTimer, opmodeTimer;
    private final Pose startPose = Config.AutoPoses.blueAutoStartPose;
    private Turret turret;
    private Indexer indexer;
    private Intake intake;
    private boolean shootCommandIncomplete;
    private boolean isBlue = true;
    MecanumDrivetrain mecanumDrivetrain = new MecanumDrivetrain(hardwareMap, startPose, isBlue);


    @Override
    public void init() {

        turret = new Turret(hardwareMap);
        indexer = new Indexer(hardwareMap);
        intake = new Intake(hardwareMap);


        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        turret.setup();
        indexer.setup();
        turret.setFlywheelTargetVelocityAsPercentage(
                Config.TurretConstants.FLYWHEEL_SPEED_HOVERING_PERCENTAGE
        );
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.debug("Shooter command complete?: " + !shootCommandIncomplete);
        panelsTelemetry.debug("Shooter rec power?: " + turret.getSpeedILUTValue(mecanumDrivetrain.getEstimatedDistanceToGoal()));
        panelsTelemetry.update(telemetry);
    }


    public static class Paths {
        public PathChain scorePreload;

        public Paths(Follower follower) {
            scorePreload = follower.pathBuilder().addPath(
                            new BezierLine(
                                    Config.AutoPoses.blueAutoStartPose,

                                    Config.AutoPoses.blueAutoEndPose
                            )
                    ).setLinearHeadingInterpolation(Config.AutoPoses.blueAutoStartPose.getHeading(),
                            Config.AutoPoses.blueAutoEndPose.getHeading())
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
                follower.followPath(paths.scorePreload);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    indexer.moveArmOut();
                    intake.intakeSpinup();
                    for (shotsFired = 0; shotsFired < 3; ++shotsFired) {
                        do {
                            shootCommandIncomplete = shootCommand();
                            Utils.halt(500);
                            } while (shootCommandIncomplete);
                        }
                    Utils.halt(1000);
                    setPathState(2);
                }
                break;
            case 2:
                turret.setFlywheelTargetVelocityAsPercentage(0);
                stop();
        }

    }
    public boolean shootCommand() {
        double distance = mecanumDrivetrain.getEstimatedDistanceToGoal();
        double targetPercentage = turret.getSpeedILUTValue(distance);
        double targetHoodAngle = turret.getHoodILUTValue(distance);

        turret.setFlywheelTargetVelocityAsPercentage(targetPercentage);
        turret.setHoodAngle(targetHoodAngle);

        //Shoot if ready
        if (turret.getFlywheelReady()) {
            indexer.moveArmOut();
            indexer.indexerForward(1);
            return false;
        }
        else if (!turret.getFlywheelReady()) {
            indexer.indexerForward(0);
            return true;
        }
        return true;
    }

    @Override
    public void stop() {
        autoEndPose = follower.getPose();
    }
}
    