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
import com.seattlesolvers.solverslib.util.Timing;

import org.firstinspires.ftc.teamcode.Main.Commands.Groups.FirePayload;
import org.firstinspires.ftc.teamcode.Main.Commands.Indexer.MoveIndexerArmInCommand;
import org.firstinspires.ftc.teamcode.Main.Commands.Indexer.UpdateIndexerState;
import org.firstinspires.ftc.teamcode.Main.Commands.Intake.ForwardIntakeCommand;
import org.firstinspires.ftc.teamcode.Main.Commands.Intake.StopIntakeCommand;
import org.firstinspires.ftc.teamcode.Main.Helpers.Drawing;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Indexer;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Main.Subsystems.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Blue 3 Ball Back", group = "!Blue Autonomous")
@Configurable // Panels
public class BlueThreeBallBack extends OpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState = 0;
    private Paths paths;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private final Pose startPose = new Pose(87.486, 8.695, Math.toRadians(90)).mirror();
    private Shooter shooter;
    private Indexer indexer;
    private Intake intake;
    private boolean shootCommandIncomplete;
    private int shotsFired = 0;
    boolean isBlue = true; // Fixed: was false
    MecanumDrivetrain mecanumDrivetrain;
    Turret turret;
    public static Pose endPose;
    public Timing.Timer firstTimer;

    FirePayload firstSalvo;

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
        follower.setMaxPower(.5);

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        paths = new Paths(follower);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        firstSalvo = new FirePayload(intake, indexer, shooter, turret, mecanumDrivetrain);
    }

    @Override
    public void start() {
        shooter.setup();
        indexer.setup();

        shooter.setFlywheelTargetVelocityAsPercentage(160);
        CommandScheduler.getInstance().schedule(new UpdateIndexerState(indexer, intake, shooter));

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
        panelsTelemetry.addLine("");
        panelsTelemetry.debug("Shooter rec power?: " + shooter.getSpeedILUTValue(mecanumDrivetrain.getEstimatedDistanceToGoal()));
        panelsTelemetry.addLine("Shooter rec hood?: " + shooter.getHoodILUTValue(mecanumDrivetrain.getEstimatedDistanceToGoal()));
        panelsTelemetry.update(telemetry);

        CommandScheduler.getInstance().run();
    }

    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(87.023, 8.695).mirror(),
                                    new Pose(87.023, 13.543).mirror()
                            )
                    ).setLinearHeadingInterpolation(
                            new Pose(0, 0, Math.toRadians(90)).mirror().getHeading(),
                            new Pose(0, 0, Math.toRadians(63)).mirror().getHeading()
                    )
                    .build();

            // Fixed: mirror Path2 poses for blue side
            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(87.023, 13.543).mirror(),
                                    new Pose(111.167, 14.103).mirror()
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
                    CommandScheduler.getInstance().schedule(new ForwardIntakeCommand(intake));
                    CommandScheduler.getInstance().schedule(firstSalvo);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy() && !CommandScheduler.getInstance().isScheduled(firstSalvo)) {
                    follower.followPath(paths.Path2);
                    CommandScheduler.getInstance().schedule(new MoveIndexerArmInCommand(indexer));
                    CommandScheduler.getInstance().schedule(new StopIntakeCommand(intake));
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    shooter.setFlywheelTargetVelocityAsPercentage(0);
                    setPathState(4); // Fixed: advance to terminal state
                }
                break;
            case 4:
                // Terminal state — do nothing
                break;
        }
    }
}