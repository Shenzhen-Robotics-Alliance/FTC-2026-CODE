package org.firstinspires.ftc.teamcode.autos;

import static org.firstinspires.ftc.teamcode.autos.AutoUtils.scoreShortBallsPose;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.RobotContainer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class blueFarAutoSix implements Auto{
    @Override
    public Command getAutonomousCommands(RobotContainer robotContainer) {
        SequentialCommandGroup sequence = new SequentialCommandGroup();

        //  0: Reset Odometry
        sequence.addCommands(new InstantCommand(() -> robotContainer.driveSubsystem.setPose(new Pose2d())));

        // <-- Step 1: Score preloaded Balls and shoot -->
        sequence.addCommands(AutoUtils.BlueDriveToFarPoseAndShot(robotContainer));

        // <-- Step 2: Intake and Score the third line balls -->
        Command driveToThirdLine = robotContainer.driveSubsystem.followPath(
                new Pose2d(scoreShortBallsPose.getTranslation(), Rotation2d.fromDegrees(0)),
                new Translation2d[]{},  //need
                new Pose2d(bluePositions.LINE_3_RIGHT_BALL,Rotation2d.fromDegrees(90)),
                Rotation2d.fromDegrees(0),
                0.3
        );
        sequence.addCommands(driveToThirdLine
                .withTimeout(1500));
        sequence.addCommands(AutoUtils.BlueDriveToIntakeThirdLineContinuousLy(robotContainer).withTimeout(1200));
        sequence.addCommands(AutoUtils.BlueFirstLineDriveToShortPoseAndShot(robotContainer, bluePositions.LINE_3_ENDING,1500));


        return sequence;
    }
}
