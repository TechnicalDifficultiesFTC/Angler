package org.firstinspires.ftc.teamcode.Main.Commands.Groups;

import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Main.Commands.Indexer.MoveIndexerArmInCommand;
import org.firstinspires.ftc.teamcode.Main.Helpers.Config;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Indexer;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Main.Subsystems.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Turret;

/**
 * Calls FireOnce 3 times to fire a full cargo of artifacts
 */
public class FirePayload extends SequentialCommandGroup {
    Shooter shooter;
    Indexer indexer;
    public FirePayload(Intake intake, Indexer indexer,
                       Shooter shooter, Turret turret,
                       MecanumDrivetrain mecanumDrivetrain) {
        this.shooter = shooter;
        this.indexer = indexer;
        addCommands(
                new FireOnce(intake,indexer,shooter, turret, mecanumDrivetrain),
                new FireOnce(intake,indexer,shooter, turret, mecanumDrivetrain),
                new FireOnce(intake,indexer,shooter, turret, mecanumDrivetrain)
        );
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        super.execute();
    }

    public boolean isFinished() {
        return super.isFinished();
    }

    @Override
    public void end(boolean interrupted) {

    }
}