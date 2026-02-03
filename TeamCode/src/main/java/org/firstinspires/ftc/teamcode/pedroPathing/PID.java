package org.firstinspires.ftc.teamcode.pedroPathing;


import com.bylazar.configurables.annotations.Configurable;
/*
This class is really useful for tuning anything (in this case flywheel and position PIDs.
Denoting the class with @Configurable and making these variables public and static allow them to be temporarily adjusted
in panels. By connecting to the robots wifi and going to the panels ip (found in the pedro pathing or panels docs)
you can temporarily adjust these values without re-uploading code. The values will only save if they are uploaded
before you initialize an opmode but will save until the robot turns off, resets, or (maybe) disconnects.

AFTER TUNING AND BEFORE TURNING THE ROBOT OFF YOU HAVE TO PUT THE CORRECT VALUES FROM PANELS INTO THIS ACTUAL CODE
Gamespecific opmodes will reference this class a lot.
 */




@Configurable
public class PID {
    public static double fly1P = 0.012;
    public static double fly1I = 0;
    public static double fly1D = 0;
    public static double fly1F = 0.0035;
    public static double fly1S = 0.8;

    public static double fly2P = 0.0018;
    public static double fly2I = 0.0002;
    public static double fly2D = 0;
    public static double fly2F = 0.0005;

    public static double autoaimP = 0.15;
    public static double autoaimI = 0.0001;
    public static double autoaimD = 0.005;
    public static double autoaimF = 0;


    public static double intakeP = 0.0006;
    public static double intakeI = 0;
    public static double intakeD = 0;
    public static double intakeF = 0.004;
    public static double intakeS = 2.5;

    public static double LL_P = 0.039;
    public static double LL_I = 0.005;

    public static double LL_D = 0.0001;


}
