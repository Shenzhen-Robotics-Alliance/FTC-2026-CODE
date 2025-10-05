package org.firstinspires.ftc.teamcode.subsystems.SuperStructure;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import edu.wpi.first.math.controller.PIDController;

public class RotSubsystem extends SubsystemBase {
    public DcMotorEx RotMotor;
    public DcMotor encoder;
    public final double maximumPower =1;
    public final double minimumPower = -1;
    public  double targetVelocity = 0 ;
    private PIDController velocityController;
    public static final double CLOCKWISE_SPEED = 1800;  //clockwise speed
    public static final double COUNTER_CLOCKWISE_SPEED = -1600;  //counter clockwise speed
    public static final double STOP_SPEED = 0;  // Stop
    public static double Kp = 0.05, Ki = 0.1, Kd = 0.002, Kf = 0.01;


    public RotSubsystem(HardwareMap hardwareMap){
        this.RotMotor = hardwareMap.get(DcMotorEx.class,"RotMotor");
        RotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.encoder = hardwareMap.get(DcMotor.class,"RotMotor");
        this.RotMotor.setDirection(DcMotorSimple.Direction.REVERSE); //reverse if needed

        this.velocityController = new PIDController(0,0,0);
    }

    public void periodic(){
        updatePidControl();
    }

    public void updatePidControl(){
        double currentVelocity = RotMotor.getVelocity();

        double pidOutput = velocityController.calculate(currentVelocity,targetVelocity);

        double feedforward = Kf * targetVelocity;

        //totalOutput = feedforward + pidOutput
        double totalOutput = pidOutput + feedforward;

        //restrict the range of output
        totalOutput = Range.clip(totalOutput,minimumPower,maximumPower);

        RotMotor.setPower(totalOutput);
    }

    public void setRotStop(){
        RotMotor.setPower(0);
    }

    public void setTargetVelocity(double velocity) {
        // reset pid parameters when face giant changes
        if (Math.abs(velocity - targetVelocity) > Math.abs(targetVelocity) * 0.5) {
            velocityController.reset();
        }
        this.targetVelocity = velocity;
    }

    // get current velocity
    public double getCurrentVelocity() {
        return RotMotor.getVelocity();
    }

    //get target velocity
    public double getTargetVelocity() {
        return targetVelocity;
    }

    //get errors
    public double getError() {
        return targetVelocity - getCurrentVelocity();
    }

    // ====== PID parameter management ======
    public void setPIDParameters(double kP, double kI, double kD, double kF) {
        velocityController.setPID(kP, kI, kD);
        this.Kf = kF;
    }
    public void resetPID() {
        velocityController.reset();
    }

}
