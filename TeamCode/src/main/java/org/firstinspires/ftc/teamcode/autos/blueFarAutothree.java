package org.firstinspires.ftc.teamcode.autos;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.RobotContainer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class blueFarAutothree implements Auto{
    @Override
    public Command getAutonomousCommands(RobotContainer robotContainer) {
        SequentialCommandGroup sequence = new SequentialCommandGroup();

        //  0: Reset Odometry
        sequence.addCommands(new InstantCommand(() -> robotContainer.driveSubsystem.setPose(new Pose2d())));

        // move to pre
        Command drive = robotContainer.driveSubsystem.followPath(
                new Pose2d(new Translation2d(0,0),Rotation2d.fromDegrees(0)),
                new Translation2d[]{},
                new Pose2d(new Translation2d(-0.03,-0.25),Rotation2d.fromDegrees(0)), //need
                Rotation2d.fromDegrees(0),
                0.75
        ).withTimeout(800);
        sequence.addCommands(drive);

        // <-- Step 2: Score preloaded Balls and shoot -->
        sequence.addCommands(AutoUtils.BlueFarPoseShot(robotContainer));

        //move
        Command drive2 = robotContainer.driveSubsystem.followPath(
                new Pose2d(new Translation2d(-0.03,-0.25),Rotation2d.fromDegrees(0)),
                new Translation2d[]{},
                new Pose2d(new Translation2d(-1.33,-0.13),Rotation2d.fromDegrees(0)),
                Rotation2d.fromDegrees(0),
                0.7).withTimeout(1500);


        return sequence;
    }
}
