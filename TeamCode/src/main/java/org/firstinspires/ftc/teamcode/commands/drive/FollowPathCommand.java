package org.firstinspires.ftc.teamcode.commands.drive;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.util.Timing;

import java.util.function.BooleanSupplier;

public class FollowPathCommand extends CommandBase {
    private Timing.Timer timer;
    private final Trajectory trajectory;

    public FollowPathCommand(Trajectory trajectory, BooleanSupplier shouldFlipPath) {
        this.trajectory = trajectory;
    }
}
