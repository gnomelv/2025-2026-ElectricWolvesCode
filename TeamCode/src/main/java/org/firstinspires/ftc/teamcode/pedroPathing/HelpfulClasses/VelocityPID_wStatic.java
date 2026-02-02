
package org.firstinspires.ftc.teamcode.pedroPathing.HelpfulClasses;

import com.qualcomm.robotcore.util.ElapsedTime;

/*
NEVER DO SKIP MODE. HONESTLY IT SHOULD BE REMOVED FROM THE CODE. WHEN ITS ON IT JUST BREAKS
This class can be used to get a motor to spin at a constant velocity and correct itself to stay at that speed
If this class is controlling two motors that are mechanically linked (through gears or belts) only use one PID.
Select one motor to be the leader and go based of its encoder. Just send the same power to both motors. With two PIDs
the motors fight each other.
Its very important for it to be tuned correctly
How to tune:
- Tune the static bit first: Set the motor to run to an arbitrary speed and increase S until the motor moves. Go slightly less than that.
the motor should be trying to move but not actually move
- Tune F second: Set the motor to try and run to a speed close to the one it'll be actually going to. Increase F until the
motor gets to around 70-80% of the target speed. F does most of the work everything else is fine tuning.
- Tune P: Increase P until the motor speed reaches its target. It'll oscillate a bit but not more than 20 ticks per second
probably.
- Tune D: Increase D so it reaches the correct speed faster and more accurately. if you can't get P accurate, then D will do it
It'll be crazy if its too high
- Tune I: Might not be needed. Just increase it slightly. it'll make the motor reach its correct speed faster

Things to improve:
- just get rid of skip mode. it sucks and isn't needed
 */


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