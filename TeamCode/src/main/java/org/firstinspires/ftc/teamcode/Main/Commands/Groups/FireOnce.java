package org.firstinspires.ftc.teamcode.Main.Commands.Groups;

import com.qualcomm.hardware.ams.AMSColorSensor;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelDeadlineGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.util.Timing;

import org.firstinspires.ftc.teamcode.Main.Commands.Indexer.UpdateIndexerState;
import org.firstinspires.ftc.teamcode.Main.Commands.Indexer.MoveIndexerArmInCommand;
import org.firstinspires.ftc.teamcode.Main.Commands.Indexer.MoveIndexerArmOutCommand;
import org.firstinspires.ftc.teamcode.Main.Commands.Intake.ForwardIntakeCommand;
import org.firstinspires.ftc.teamcode.Main.Helpers.ShooterTracker;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Indexer;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Main.Subsystems.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Turret;

import java.util.concurrent.TimeUnit;

/**
 * Fires one ball, end condition monitors for interial drop from shooting an artifact
 */
public class FireOnce extends SequentialCommandGroup {
    Shooter shooter;
    Indexer indexer;
    Intake intake;
    ShooterTracker shooterTracker;
    boolean shotFired;
    Timing.Timer timer;
    public FireOnce(Intake intake, Indexer indexer, Shooter shooter, Turret turret, MecanumDrivetrain mecanumDrivetrain) {
        WaitCommand waitCommand;
        if (mecanumDrivetrain.getEstimatedDistanceToGoal() > 100) {
            waitCommand = new WaitCommand(450);
        } else {
            waitCommand = new WaitCommand(250);
        }
        addCommands(
                new ParallelCommandGroup(
                        new PrimeShooter(shooter, mecanumDrivetrain),
                        new WaitUntilCommand(turret::isTurretReady)
                ),
                new ParallelDeadlineGroup(
                        new MoveIndexerArmOutCommand(indexer),
                        new ForwardIntakeCommand(intake)
                ),
                waitCommand
        );
    }

    public void initialize() {
        //Start first command
        super.initialize();
    }

    public void execute() {
        //execute command list defined in command construction
        super.execute();
    }

    //End shooterTracker period
    public boolean isFinished() {
        //return shotFired;
        return super.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
    }
}