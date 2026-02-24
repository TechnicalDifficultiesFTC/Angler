package org.firstinspires.ftc.teamcode.Main.Commands.Indexer;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.util.Timing;

import org.firstinspires.ftc.teamcode.Main.Helpers.Config;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Indexer;

import java.util.concurrent.TimeUnit;

/**
 * Practically the only difference is the method change from indexer.moveArmIn to
 * indexer.moveArmOut
 */
public class MoveIndexerArmOutCommand extends CommandBase {
    Indexer indexer;
    Timing.Timer timer = new Timing.Timer(Config.IndexerConstants.TRANSITION_TIME_MILLISECONDS,
            TimeUnit.MILLISECONDS);
    public MoveIndexerArmOutCommand(Indexer indexer) {
        this.indexer = indexer;
        //no requirements are specified here intentionally
    }

    public void initialize() {
        indexer.moveArmOut();
        timer.start();
    }

    public boolean isFinished () {
        return timer.done();
    }

}
