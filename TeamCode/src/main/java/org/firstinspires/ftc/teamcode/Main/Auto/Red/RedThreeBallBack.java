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

@Autonomous(name = "Red 3 Ball Back UPD", group = "!Red Autonomous")
@Configurable // Panels
public class RedThreeBallBack extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState = 0; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private Timer pathTimer, actionTimer, opmodeTimer;
    private final Pose startPose = new Pose(87.023, 8.695, Math.toRadians(90));
    private Shooter shooter;
    private Indexer indexer;
    private Intake intake;
    private boolean shootCommandIncomplete;
    private int shotsFired = 0;
    boolean isBlue = false;
    MecanumDrivetrain mecanumDrivetrain;
    Turret turret;
    public static Pose endPose;
    public Timing.Timer firstTimer;

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
        follower.setMaxPower(.5);

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        firstSalvo = new FirePayload(intake,indexer,shooter, turret, mecanumDrivetrain);
        secondSalvo = new FirePayload(intake,indexer,shooter, turret, mecanumDrivetrain);
    }

    @Override
    public void start() {
        shooter.setup();
        indexer.setup();

        shooter.setFlywheelTargetVelocityAsPercentage(160);
        CommandScheduler.getInstance().schedule(new UpdateIndexerState(indexer,intake,shooter));

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

        CommandScheduler.getInstance().run();
    }








    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(87.023, 8.695),

                                    new Pose(87.023, 13.543)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(63))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(87.023, 13.543),

                                    new Pose(131.077, 12.251)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))

                    .build();
        }
    }



    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
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
                    CommandScheduler.getInstance().schedule(new MoveIndexerArmInCommand(indexer));
                    CommandScheduler.getInstance().schedule(new StopIntakeCommand(intake));
                    follower.followPath(paths.Path2);
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
