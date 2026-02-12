package org.firstinspires.ftc.teamcode.Main.Commands.Indexer;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Main.Subsystems.Indexer;

public class ReverseIndexer extends CommandBase {
    Indexer indexer;

    public ReverseIndexer(Indexer indexer) {
        this.indexer = indexer;
    }

    @Override
    public void initialize() {
        indexer.indexerReverse();
    }

    public boolean isFinished() {
        return true;
    }
}
