package org.firstinspires.ftc.teamcode.subsystems.SuperStructure;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.constants.SystemConstants;
import org.firstinspires.ftc.teamcode.utils.SuperStructure.SimpleMechanism;

public class LinearMotion implements SimpleMechanism, Subsystem {
    public final String name;

    private final DcMotorEx[] motors;
    private final boolean[] motorsReversed;
    private final DcMotorEx encoder;
    private final boolean encoderReversed;
    private final double maximumSpeed;
    private final PIDFController controller;
    //private final double kG, kV, kS;
    private final double kp, ki, kd,kf;
    private double setPoint;

    private double previousError = 0;
    private double  integralSum = 0;
    private double targetVelocity = 0;

    private double previousSetPoint = 0;

    public LinearMotion(
            String name,
            DcMotorEx[] motors, boolean[] motorsReversed,
            DcMotorEx encoder, boolean encoderReversed,
            double maximumSpeed,
            double kP,
            double kI,
            double kD,
            double kF
           // double kG,double kV,double kS, double kP
    ){
        this.name = name;

        this.motors = motors;
        this.motorsReversed = motorsReversed;
        this.encoder = encoder;
        this.encoderReversed = encoderReversed;
        this.maximumSpeed = maximumSpeed;
        this.kp = kP;
        this.ki = kI;
        this.kd = kD;
        this.kf = kF;
        this.controller = new PIDFController(kP,0,0,0);


        for (DcMotor motor:motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        encoder.setDirection(DcMotorSimple.Direction.FORWARD);

        this.targetVelocity = 0;  //initalize

    }


    private double encoderZeroPosition = 0.0;

    //(-1.0 to 1.0)
    public void setTargetVelocity(double targetVelocity) {
        this.targetVelocity = targetVelocity;
    }

    // Get current velocity from the encoder

    public double getCurrentVelocity() {
        double rawVelocity = encoder.getVelocity();
        double normalizedVelocity = rawVelocity / maximumSpeed;
        return encoderReversed ? -normalizedVelocity : normalizedVelocity;
    }

    public void resetController() {
        this.integralSum = 0;
        this.previousError = 0;
        this.targetVelocity = 0; // 将目标速度也设为0
    }

    public void periodic(){

        double currentVelocity = getCurrentVelocity();
      //  double desiredVelocity = (setPoint - previousSetPoint) * SystemConstants.ROBOT_UPDATE_RATE_HZ;

        previousSetPoint = setPoint;

        double error = targetVelocity - currentVelocity;

        //proportional term
        double pTerm = error * kp;

        // Integral Term
        integralSum += error * (1.0 / SystemConstants.ROBOT_UPDATE_RATE_HZ);
        double iTerm = integralSum * ki;

        // D Term
        // 4. Derivative Term
        double derivative = (error - previousError) / (1.0 / SystemConstants.ROBOT_UPDATE_RATE_HZ);
        double dTerm = derivative * kd;
        previousError = error; // update

        double fTerm = targetVelocity * kf;

        double totalPower = pTerm + iTerm + dTerm + fTerm;

        totalPower = Math.min(Math.max(totalPower, -1.0), 1.0);

        runPower(totalPower);
    }

    public void runPower(double power){
        for (int i = 0; i < motors.length; i++) {
            motors[i].setPower((power) * (motorsReversed[i] ? -1:1));
        }
    }

    @Override
    public void goToPosition(double setPoint) {
        this.setPoint = setPoint;
    }

    public double getCurrentSetPoint() {
        return this.setPoint;
    }

    public void setMotorsStop(){
        for (int i = 0; i < motors.length; i++) {
            motors[i].setPower(0);
        }
    }
    public double getTargetVelocity() {
        return targetVelocity;
    }

    public double getCurrentVelocityRaw() {
        return encoder.getVelocity(); // original RPM
    }

    public double getVelocityError() {
        return targetVelocity - getCurrentVelocity();
    }

    public double getOutputPower() {
        // 计算当前所有马达的平均功率
        double total = 0;
        for (DcMotorEx motor : motors) {
            total += motor.getPower();
        }
        return total / motors.length;
    }

    //for rotate
    public double getPosition() {
        return encoder.getCurrentPosition();
    }

}
