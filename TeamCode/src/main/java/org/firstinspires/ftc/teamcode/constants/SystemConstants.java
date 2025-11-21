package org.firstinspires.ftc.teamcode.constants;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.EmptyTelemetry;


public class SystemConstants {
    public static long matchStartTimeMillis = System.currentTimeMillis();
    public static Telemetry telemetry = new EmptyTelemetry();
    public static final double ROBOT_UPDATE_RATE_HZ = 50.0;
    public static final double ODOMETRY_UPDATE_RATE_HZ = 200.0;
    public static final double IMU_UPDATE_HZ = 5;
    public static final double TARGET_HIGHT_CM = 90.45;
    public static final double CAMERA_ANGLE_DEGREE = 66;
    public static final double CAMERA_HIEHT_CM = 25.4;
    public static final double ROT_SOFT_LIMIT = 234;


}
