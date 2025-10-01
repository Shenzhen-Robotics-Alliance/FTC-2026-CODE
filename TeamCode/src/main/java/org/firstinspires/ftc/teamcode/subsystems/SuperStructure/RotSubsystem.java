package org.firstinspires.ftc.teamcode.subsystems.SuperStructure;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RotSubsystem extends SubsystemBase {
    public DcMotor RotMotor;
    public DcMotor encoder;
    public final double maximumPower =1;
    public final double minimumPower = -1;
    public  double targetVelocity = 0 ;

    public static final double CLOCKWISE_SPEED = 1800;  //clockwise speed
    public static final double COUNTER_CLOCKWISE_SPEED = -1600;  //counter clockwise speed
    public static final double STOP_SPEED = 0;  // Stop

    public void init(HardwareMap hardwareMap){
        this.RotMotor = hardwareMap.get(DcMotor.class,"RotMotor");
        RotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.encoder = hardwareMap.get(DcMotor.class,"RotMotor");
        this.RotMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }
    

}
