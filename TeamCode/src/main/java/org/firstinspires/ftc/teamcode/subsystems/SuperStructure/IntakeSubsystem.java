package org.firstinspires.ftc.teamcode.subsystems.SuperStructure;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeSubsystem extends SubsystemBase{
    public  Motor intakeMotor;
    private Telemetry telemetry;
    private  DcMotor encoder;
    private boolean isMotorInverted = false;
    private double error = 0, lastError = 0, integral = 0, derivative = 0;
    public static double Kp = 0, Ki = 0, Kd = 0, power;
    private ElapsedTime timer = new ElapsedTime();

    public void init(HardwareMap hardwareMap){
        intakeMotor = hardwareMap.get(MotorEx.class,"intakeMotor");
        intakeMotor.setInverted(isMotorInverted);
        intakeMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        encoder = hardwareMap.get(DcMotor.class,"intakeMotor");
    }

    public void setIntakePower(){
        
    }


}