package org.firstinspires.ftc.teamcode.Main.Commands.Groups;

import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Main.Subsystems.Indexer;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Main.Subsystems.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Shooter;

public class FirePayload extends SequentialCommandGroup {
    public FirePayload(Intake intake, Indexer indexer, Shooter shooter, MecanumDrivetrain mecanumDrivetrain) {
        addCommands(
                new FireOnce(intake,indexer,shooter,mecanumDrivetrain),
                new FireOnce(intake,indexer,shooter,mecanumDrivetrain),
                new FireOnce(intake,indexer,shooter,mecanumDrivetrain)
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
}