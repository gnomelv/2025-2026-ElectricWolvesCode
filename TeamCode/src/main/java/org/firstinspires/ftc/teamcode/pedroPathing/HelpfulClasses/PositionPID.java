package org.firstinspires.ftc.teamcode.pedroPathing.HelpfulClasses;

import com.qualcomm.robotcore.util.ElapsedTime;

/*
NEVER DO SKIP MODE. HONESTLY JUST REMOVE IT WHEN YOU'RE WORKING ON THIS CLASS
This class works for running a motor to a given position
It needs to be tuned accurately and the target and current values in input should always be in the same reference frame
Steps for tuning:
- Tune P first. Increase P slightly until the motor moves to the correct position. It should oscillate constantly around the
 target when P is too large. It should be just large enough to reach the position fast and oscillate a bit but eventually stop
 -Tune D second: Increase D to make it oscillate less and be more accurate. It'll vibrate/pscillate like crazy if D is too big
 Tune I last: If the target is stopping close to the error but not right at it, it could be because the error is
 so small that P can't overcome some resistance from gears or internally or something. This is what I is for. Increase
 it slightly until this issue goes away. The value should be quite small

Things to possibly do:
- Integrate input voltage. The velocity PIDs do this and it would probably be similar and easy to integrate with zero downside
- Static coefficeint: I'm not sure this is needed. If you determine that it is, it'll probably do a lot of the work that I doe
 */

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
