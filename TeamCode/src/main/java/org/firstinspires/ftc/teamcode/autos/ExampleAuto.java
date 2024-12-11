package org.firstinspires.ftc.teamcode.autos;


import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.RobotContainer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;


public class ExampleAuto implements Auto {
    @Override
    public Command getAutonomousCommands(RobotContainer robotContainer) {
        final SequentialCommandGroup sequence = new SequentialCommandGroup();

        sequence.addCommands(robotContainer.driveSubsystem.followPath(
                new Pose2d[]{
                        new Pose2d(0, 0, Rotation2d.fromDegrees(45)),
                        new Pose2d(0.5, 0.5, Rotation2d.fromDegrees(90)),
                        new Pose2d(0.5, 1, Rotation2d.fromDegrees(90))},
                Rotation2d.fromDegrees(90),
                1));

        sequence.addCommands(new WaitCommand(5000));

        return sequence;
    }
}
