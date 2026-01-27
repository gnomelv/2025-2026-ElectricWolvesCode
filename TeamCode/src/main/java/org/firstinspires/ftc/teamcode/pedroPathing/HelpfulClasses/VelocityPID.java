
package org.firstinspires.ftc.teamcode.pedroPathing.HelpfulClasses;

import com.qualcomm.robotcore.util.ElapsedTime;

public class VelocityPID {

    private double integralSum = 0;
    private double lastError = 0;
    private double lastTarget = 0;

    private double Kp, Ki, Kd, Kv;
    public double error;

    private boolean skip;
    private int timesSkipped = 1;

    private ElapsedTime timer = new ElapsedTime();

    public VelocityPID(double Kp_Input, double Ki_Input, double Kd_Input, double FInput, boolean skipMode) {
        Kp = Kp_Input;
        Ki = Ki_Input;
        Kd = Kd_Input;
        Kv = FInput; // Feedforward (volts per velocity unit)
        skip = skipMode;
    }

    /**
     * @param target target velocity (ticks/sec, RPM, etc.)
     * @param current current velocity (same units as target)
     * @param batteryVoltage current robot battery voltage
     * @return motor power command (-1.0 to 1.0)
     */
    public double update(double target, double current, double batteryVoltage) {

        double outputVolts;

        if (timesSkipped == 1) {

            error = target - current;

            double dt = timer.seconds();
            if (dt <= 0) dt = 1e-6;

            double derivative = (error - lastError) / dt;
            integralSum += error * dt;

            // Voltage-based PIDF
            outputVolts =
                    (Kp * error) +
                            (Ki * integralSum) +
                            (Kd * derivative) +
                            (Kv * target);

            lastError = error;
            timer.reset();

            if (target != lastTarget) {
                integralSum = 0; // anti-windup on target change
            }
            lastTarget = target;

            if (skip) {
                timesSkipped = 0;
            }

        } else {
            timesSkipped++;
            outputVolts = 0;
        }

        // Convert volts → normalized motor power
        double power = outputVolts / batteryVoltage;

        // Clamp AFTER voltage normalization
        power = Math.max(-1.0, Math.min(1.0, power));

        return power;
    }
}