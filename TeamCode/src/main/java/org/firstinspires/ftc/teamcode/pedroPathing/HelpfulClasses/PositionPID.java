package org.firstinspires.ftc.teamcode.pedroPathing.HelpfulClasses;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PositionPID {

    private double Kp, Ki, Kd;
    private double integralSum = 0;
    public double error = 0;
    private double lastError = 0;
    private double lastTarget = 0;

    private double maxIntegral = 0.5; //so it doesn't kill itself
    private ElapsedTime timer = new ElapsedTime();

    public PositionPID(double kp, double ki, double kd) {
        Kp = kp;
        Ki = ki;
        Kd = kd;
    }

    public double update(double target, double current) {



        double dt = timer.seconds();
        timer.reset();
        if (dt <= 0) dt = 1e-3;


        //angle wrapping
        error = target - current;
        while (error > 180) error -= 360;
        while (error < -180) error += 360;


        if (Math.signum(error) != Math.signum(lastError)) {
            integralSum = 0;
        }

        integralSum += error * dt;
        integralSum = Math.max(-maxIntegral, Math.min(maxIntegral, integralSum));
        if (target != lastTarget){integralSum=0;}

        double derivative = (error - lastError) / dt;

        lastError = error;

        double out = ((Kp * error) + (Ki * integralSum) + (Kd * derivative));

        if (Math.abs(error) < 0.4) out = 0;

        return out;
    }

    public void reset() {
        integralSum = 0;
        lastError = 0;
        timer.reset();
    }
}
