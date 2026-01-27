
package org.firstinspires.ftc.teamcode.pedroPathing.HelpfulClasses;

import com.qualcomm.robotcore.util.ElapsedTime;

public class VelocityPID_wStatic {

    private double integralSum = 0;
    private double lastError = 0;
    private double lastTarget = 0;

    private double Kp, Ki, Kd, Kv, Ks;
    public double error;

    private boolean skip;
    private int timesSkipped = 1;

    private ElapsedTime timer = new ElapsedTime();

    public VelocityPID_wStatic(double Kp_Input, double Ki_Input, double Kd_Input, double FInput, double Ks_input, boolean skipMode) {
        Kp = Kp_Input;
        Ki = Ki_Input;
        Kd = Kd_Input;
        Kv = FInput; // use this
        Ks = Ks_input;
        skip = skipMode;
    }

    public double update(double target, double current, double batteryVoltage) {

        double outputVolts;

        if (timesSkipped == 1) {

            error = target - current;

            double dt = timer.seconds();
            if (dt <= 0) dt = 1e-6;

            double derivative = (error - lastError) / dt;
            integralSum += error * dt;

            double staticFF = 0.0;
            if (Math.abs(target) > 1e-3) {
                staticFF = Ks * Math.signum(target);
            }

            outputVolts =
                    (Kp * error) +
                            (Ki * integralSum) +
                            (Kd * derivative) +
                            staticFF +
                            (Kv * target);

            lastError = error;
            timer.reset();

            if (target != lastTarget) {
                integralSum = 0;
            }
            lastTarget = target;

            if (skip) {
                timesSkipped = 0;
            }

        } else {
            timesSkipped++;
            outputVolts = 0;
        }

        double power = outputVolts / batteryVoltage;
        power = Math.max(-1.0, Math.min(1.0, power));

        return power;
    }
}