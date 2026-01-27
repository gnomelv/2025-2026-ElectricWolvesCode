package org.firstinspires.ftc.teamcode.pedroPathing;


import com.bylazar.configurables.annotations.Configurable;

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
