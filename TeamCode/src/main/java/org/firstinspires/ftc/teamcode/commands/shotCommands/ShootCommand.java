package org.firstinspires.ftc.teamcode.commands.shotCommands;
//=========1,shootCommand============
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.subsystems.SuperStructure.PreShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SuperStructure.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.utils.Interpolator;

import edu.wpi.first.math.geometry.Pose2d;

public class ShootCommand extends CommandBase {
    private ShooterSubsystem shooterSubsystem;
    private PreShooterSubsystem preShooterSubsystem;
    private final MecanumDriveSubsystem driveSubsystem;
    private final Interpolator rpmTable;
    private double currentTPS = 0;


    //HIgh goal position
    private static final double GOAL_X_METERS = 0;
    private static final double GOAL_Y_METERS = 0;


    public ShootCommand(ShooterSubsystem shooterSubsystem,PreShooterSubsystem preShooterSubsystem,MecanumDriveSubsystem driveSubsystem){
        this.shooterSubsystem = shooterSubsystem;
        this.preShooterSubsystem = preShooterSubsystem;
        this.driveSubsystem = driveSubsystem;
        addRequirements(shooterSubsystem);

        this.rpmTable = new Interpolator();
        rpmTable.add(0.5, 1350); //distance unit: meters
        rpmTable.add(0.6, 1500);
        rpmTable.add(0.7, 1680);
    }

    @Override
    public void initialize(){
        preShooterSubsystem.setStopPreShooter();
    }
    // <Fixed Point Shooting in both Far and Short point>
    public Command fixShootFarContinuously() {
        return new ParallelCommandGroup(
                shooterSubsystem.shooterFixFarLaunch(),

                new SequentialCommandGroup(
                        new WaitUntilCommand(shooterSubsystem::isReadyToFixFarLaunch),
                        preShooterSubsystem.setShootingAngle()
                )
        );
    }

    public Command fixShootShortContinuously() {
        return new ParallelCommandGroup(
                shooterSubsystem.shooterFixShortLaunch(),

                new SequentialCommandGroup(
                        new WaitUntilCommand(shooterSubsystem::isReadyToFixShortLaunch),
                        preShooterSubsystem.setShootingAngle()
                )
        );
    }

    public Command shootStop(){
        SequentialCommandGroup sequence = new SequentialCommandGroup();
        sequence.addRequirements(shooterSubsystem);

        sequence.addCommands(shooterSubsystem.setShootingMotorStop()
                .alongWith(preShooterSubsystem.setStopPreShooter()));

        return sequence;
    }

    //<Auto velocity shooting according to current pose>
    public Command shootAutoVelocity() {
        return new ParallelCommandGroup(
                shooterSubsystem.shooterAutoLaunch(),

                new SequentialCommandGroup(
                        new WaitUntilCommand(shooterSubsystem::isReadyToAutoLaunch),
                        preShooterSubsystem.setShootingAngle()
                )
        );
    }

}

