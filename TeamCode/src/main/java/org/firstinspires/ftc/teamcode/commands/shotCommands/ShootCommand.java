package org.firstinspires.ftc.teamcode.commands.shotCommands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
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
    public void initialize(){
        shooterSubsystem.setShooterStop();
    }

    /**
    @Override
    public void execute(){
        shooterSubsystem.setShootingVelocity(0.9);
    }
*/

    @Override
    public void end(boolean interrupted){
        shooterSubsystem.shooter.setMotorsStop();
        shooterSubsystem.setHoldBallAngle();
    }

    @Override
    public boolean isFinished(){
        return false;
    }


//    public Command shootFarContinuously(){
//        SequentialCommandGroup sequence = new SequentialCommandGroup();
//        sequence.addRequirements(shooterSubsystem);
//
//        //initialize the motors and servos
//        sequence.addCommands(
//                shooterSubsystem.setHoldBallAngle()
//                        .alongWith(shooterSubsystem.shooterStop()
//         ));
//
//        //enable the motors for far launch
//        sequence.addCommands(shooterSubsystem.shooterFarLaunch());
//
//        //enable the servo
//        sequence.addCommands(new ConditionalCommand(
//                shooterSubsystem.setFarShootingAngle(),
//                shooterSubsystem.setHoldBallAngle(),
//                () -> shooterSubsystem.isReadyToFarLaunch()
//        ));
//        return sequence;
//    }
public Command shootFarContinuously() {
    return new SequentialCommandGroup(
            // 初始化
            shooterSubsystem.setHoldBallAngle()
                    .alongWith(shooterSubsystem.shooterStop()),

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
                        .alongWith(shooterSubsystem.shooterStop()),

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

        sequence.addCommands(shooterSubsystem.shooterStop());
        sequence.addCommands(shooterSubsystem.setHoldBallAngle());

        return sequence;
    }

}

