package org.firstinspires.ftc.teamcode.Main.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Main.Subsystems.Indexer;
import org.firstinspires.ftc.teamcode.Main.Subsystems.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Turret;

public class SpinupFlywheelCommand extends CommandBase {
    private final Turret turret;
    private final MecanumDrivetrain mecanumDrivetrain;
    double distance;
    double targetPercentage;
    double targetHoodAngle;
    public SpinupFlywheelCommand(Turret turret, MecanumDrivetrain mecanumDrivetrain) {
        this.turret = turret;
        this.mecanumDrivetrain = mecanumDrivetrain;
    }

    public void initialize() {
        distance = mecanumDrivetrain.getEstimatedDistanceToGoal();
        targetPercentage = turret.getSpeedILUTValue(distance);
        targetHoodAngle = turret.getHoodILUTValue(distance);
    }
}
