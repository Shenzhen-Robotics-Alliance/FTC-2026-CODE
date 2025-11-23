package org.firstinspires.ftc.teamcode.commands.shotCommands;
//=========1,shootCommand============
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.teamcode.subsystems.SuperStructure.PreShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SuperStructure.ShooterSubsystem;

public class ShootCommand extends CommandBase {
    private ShooterSubsystem shooterSubsystem;
    private PreShooterSubsystem preShooterSubsystem;

    public ShootCommand(ShooterSubsystem shooterSubsystem,PreShooterSubsystem preShooterSubsystem){
        this.shooterSubsystem = shooterSubsystem;
        this.preShooterSubsystem = preShooterSubsystem;
        addRequirements(shooterSubsystem);
    }


    public Command shootFarContinuously() {
        SequentialCommandGroup sequence = new SequentialCommandGroup();
        sequence.addRequirements(shooterSubsystem);

        sequence.addCommands(shooterSubsystem.shooterFarLaunch()
                .alongWith(shooterSubsystem.isReadyToFarLaunch()
                        ? preShooterSubsystem.setShootingAngle()
                        : preShooterSubsystem.setStopPreShooter()));

        return sequence;
    }

    public Command shootShortContinuously(){
        SequentialCommandGroup sequence = new SequentialCommandGroup();
        sequence.addRequirements(shooterSubsystem);

        sequence.addCommands(shooterSubsystem.shooterShortLaunch()
                .alongWith(shooterSubsystem.isReadyToShortLaunch()
                        ? preShooterSubsystem.setShootingAngle()
                        : preShooterSubsystem.setStopPreShooter()));

        return sequence;
    }

    public Command shootStop(){
        SequentialCommandGroup sequence = new SequentialCommandGroup();
        sequence.addRequirements(shooterSubsystem);

        sequence.addCommands(shooterSubsystem.setShootingMotorStop()
                .alongWith(preShooterSubsystem.setStopPreShooter()));

        return sequence;
    }
}

