package org.firstinspires.ftc.teamcode.commands.shotCommands;

import com.arcrobotics.ftclib.command.CommandBase;

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

    @Override
    public void execute(){
        shooterSubsystem.setShootingVelocity(0.9);
    }


    @Override
    public void end(boolean interrupted){
        shooterSubsystem.shooter.setMotorsStop();
    }

    @Override
    public boolean isFinished(){
        return false;
    }

    /**
    public Command shootFarContinuously(){
        SequentialCommandGroup sequence = new SequentialCommandGroup();
        sequence.addRequirements(shooterSubsystem);

        //initialize the motors and servos
        sequence.addCommands(
                shooterSubsystem.setHoldBallAngle()
                        .alongWith(shooterSubsystem.shooterStop()
         ));

        //enable the servo after the motors execute
        sequence.addCommands(shooterSubsystem.shooterFarLaunch());

        //enable the servo
        sequence.addCommands(new ConditionalCommand(
                shooterSubsystem.setShootingAngle(),
                shooterSubsystem.setHoldBallAngle(),
                () -> shooterSubsystem.isReadyToLaunch()
        ));

        return sequence;
    }
*/


}

