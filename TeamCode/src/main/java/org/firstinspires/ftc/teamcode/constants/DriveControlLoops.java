package org.firstinspires.ftc.teamcode.constants;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class DriveControlLoops {
    public static final ProfiledPIDController rotationController = new ProfiledPIDController(
            5.0, 0, 0,
            new TrapezoidProfile.Constraints(DriveTrainConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, 720)
    );
    private final HolonomicDriveController driveController = new HolonomicDriveController(
            new PIDController(5.0, 0, 0),
            new PIDController(5.0, 0, 0),
            rotationController
    );
}
