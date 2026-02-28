package org.firstinspires.ftc.teamcode.Main.Commands.Shooter;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Main.Subsystems.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Shooter;

public class SpinupFlywheelCommand extends CommandBase {
    private final Shooter shooter;
    private final MecanumDrivetrain mecanumDrivetrain;
    double distance;
    double targetPercentage;
    public SpinupFlywheelCommand(Shooter shooter, MecanumDrivetrain mecanumDrivetrain) {
        this.shooter = shooter;
        this.mecanumDrivetrain = mecanumDrivetrain;
    }

    @Override
    public void initialize() {
        distance = mecanumDrivetrain.getEstimatedDistanceToGoal();
        targetPercentage = shooter.getSpeedILUTValue(distance);
        shooter.setFlywheelTargetVelocityAsPercentage(targetPercentage);
    }

    @Override
    //Continue to recalculate based on distance in case we get pushed
    public void execute() {
        distance = mecanumDrivetrain.getEstimatedDistanceToGoal();
        targetPercentage = shooter.getSpeedILUTValue(distance);
        shooter.setFlywheelTargetVelocityAsPercentage(targetPercentage);
    }

    @Override
    public boolean isFinished() {
        return shooter.isFlywheelReady();
    }
}
