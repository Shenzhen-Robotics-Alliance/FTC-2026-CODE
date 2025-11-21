package org.firstinspires.ftc.teamcode.commands.shotCommands;
//=========1,shootCommand============
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.subsystems.SuperStructure.ShooterSubsystem;

public class ShootCommand extends CommandBase {
    private ShooterSubsystem shooterSubsystem;

    public ShootCommand(ShooterSubsystem shooterSubsystem){
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        shooterSubsystem.setShooterStop();
    }

    @Override
    public void end(boolean interrupted){
        shooterSubsystem.setShooterStop();
        shooterSubsystem.setHoldBallAngle();
    }

    @Override
    public boolean isFinished(){
        return false;
    }


    public Command shootFarContinuously() {
    return new SequentialCommandGroup(
            // 初始化
            shooterSubsystem.setHoldBallAngle()
                    .alongWith(shooterSubsystem.setShootingMotorStop()),

            shooterSubsystem.shooterFarLaunch().withTimeout((long) 3.0),

            new WaitUntilCommand(shooterSubsystem::isReadyToFarLaunch)
                    .withTimeout((long)2.0)
                    .andThen(shooterSubsystem.setFarShootingAngle()),

            // 射完回中
            new InstantCommand(() -> shooterSubsystem.setHoldBallAngle())
        );
    }

    public Command shootShortContinuously(){
        return new SequentialCommandGroup(
                shooterSubsystem.setHoldBallAngle()
                        .alongWith(shooterSubsystem.setShootingMotorStop()),

                shooterSubsystem.shooterShortLaunch().withTimeout((long) 3.0),

                new WaitUntilCommand(shooterSubsystem::isReadyToShortLaunch)
                        .withTimeout((long)2.0)
                        .andThen(shooterSubsystem.setShortShootingAngle()),

                new InstantCommand(() -> shooterSubsystem.setHoldBallAngle())
        );
    }

    public Command shootStop(){

        SequentialCommandGroup sequence = new SequentialCommandGroup();
        sequence.addRequirements(shooterSubsystem);

        sequence.addCommands(shooterSubsystem.setShootingMotorStop()
                .alongWith(shooterSubsystem.setHoldBallAngle()));

        return sequence;

    }
}

