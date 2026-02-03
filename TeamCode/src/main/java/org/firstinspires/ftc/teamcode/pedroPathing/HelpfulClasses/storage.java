package org.firstinspires.ftc.teamcode.pedroPathing.HelpfulClasses;

import com.pedropathing.geometry.Pose;

/*
I never had a chance to fully test this but it should be very helpful. By designating a variable as static
it should be able to be saved when you switch opmodes. The goal is that at the end of autonomous (or periodically
throughout it) we update the teleOpStart pose and then check it at the beginning of teleop to tell the robot
where it ended the auton. startingTicks would do the same thing with the encoder ticks when the turret is pointing straight
forward.

IMPORTANT: IF THIS WORKS, THE VALUES WILL ANYWAYS RESET WHEN THE ROBOT TURNS OFF, RESETS, OR DISCONNECTS

I kept these values in their own class just for simplicity. It's probably not a great idea to imbed them inside
an opmodes class.
 */


public class storage {
    public static Pose  teleOpStart= new Pose (0,0,0);
    public static double startingTicks = 0;
}
