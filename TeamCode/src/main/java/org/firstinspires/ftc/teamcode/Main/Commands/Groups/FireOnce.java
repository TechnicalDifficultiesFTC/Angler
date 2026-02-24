package org.firstinspires.ftc.teamcode.Main.Commands.Groups;

import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

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

/**
 * Fires one ball, end condition monitors for interial drop from shooting an artifact
 */
public class FireOnce extends SequentialCommandGroup {
    Shooter shooter;
    Indexer indexer;
    Intake intake;
    ShooterTracker shooterTracker;
    boolean shotFired;
    public FireOnce(Intake intake, Indexer indexer, Shooter shooter, Turret turret, MecanumDrivetrain mecanumDrivetrain) {
        this.shooter = shooter;
        this.indexer = indexer;
        this.intake = intake;

        this.shooterTracker = shooter.getShooterTracker();
        addCommands(
                new PrimeShooter(shooter, mecanumDrivetrain),
                new ParallelCommandGroup(
                    new ForwardIntakeCommand(intake),
                    new MoveIndexerArmOutCommand(indexer)
                )
                //might have to call update indexer here
        );
    }

    public void initialize() {
        //Start first command
        super.initialize();
    }

    public void execute() {
        //execute command list defined in command construction
        super.execute();
        
        //Start shooter tracker if it hasn't been started already
        if (shooter.isFlywheelReady() && !shooterTracker.isMonitoring()) {
            double targetVel = shooter.getFlywheelTargetVelocityAsPercentage();
            shooterTracker.startMonitoring(targetVel);
        }

        //While shooter tracker is online update shotFired
        if (shooterTracker.isMonitoring()) {
            double targetVel = shooter.getFlywheelTargetVelocityAsPercentage();
            double currentVel = shooter.getFlywheelCurrentVelocityAsPercentage();
            //could send this to the shooter periodic instead but this is safer
            shooterTracker.update(currentVel,targetVel);
            shotFired = shooterTracker.shotWasFired();
        }
    }

    //End shooterTracker period
    public boolean isFinished() {
        return shotFired;
    }

    @Override
    public void end(boolean interrupted) {
        shooterTracker.stopMonitoring();
    }
}