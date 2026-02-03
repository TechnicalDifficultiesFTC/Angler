package org.firstinspires.ftc.teamcode.Main.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Main.Subsystems.Indexer;
import org.firstinspires.ftc.teamcode.Main.Subsystems.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Turret;

public class ShootCommand extends CommandBase {
    private final Turret turret;
    private final MecanumDrivetrain mecanumDrivetrain;
    public ShootCommand(Turret turret, Indexer indexer, MecanumDrivetrain mecanumDrivetrain) {
        this.turret = turret;
        this.mecanumDrivetrain = mecanumDrivetrain;
    }

    public void initialize() {
        double distance = mecanumDrivetrain.getEstimatedDistanceToGoal();
        double targetPercentage = turret.getSpeedILUTValue(distance);
        double targetHoodAngle = turret.getHoodILUTValue(distance);

    }
}
