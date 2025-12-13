package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;

@Config
public class PIDFConstants {
    public static double TOP_SHOOTER_P = 0.005;
    public static double TOP_SHOOTER_I = 0.0;
    public static double TOP_SHOOTER_D = 0.0;
    public static double TOP_SHOOTER_Kv = 0.00039;
    public static double TOP_SHOOTER_Ks = 0.07;

    public static double BOTTOM_SHOOTER_P = 0.004;
    public static double BOTTOM_SHOOTER_I = 0.0;
    public static double BOTTOM_SHOOTER_D = 0.001;
    public static double BOTTOM_SHOOTER_Kv = 0.00047;
    public static double BOTTOM_SHOOTER_Ks = 0;
    /*public static double TOP_SHOOTER_P = 0.012;
    public static double TOP_SHOOTER_I = 0.0;
    public static double TOP_SHOOTER_D = 0.01;
    public static double TOP_SHOOTER_Kv = 0.0004;
    public static double TOP_SHOOTER_Ks = 0.14;

    public static double BOTTOM_SHOOTER_P = 0.0041;//0.004;
    public static double BOTTOM_SHOOTER_I = 0.0;
    public static double BOTTOM_SHOOTER_D = 0.00;//0.001;
    public static double BOTTOM_SHOOTER_Kv = 0.0004;//0.0005;
    public static double BOTTOM_SHOOTER_Ks = 0.09;*/

    public static double INTAKE_P = 0.01;
    public static double INTAKE_I = 0;
    public static double INTAKE_D = 0.00025;

    public static double HEADING_P = 0.04; //0.025
    public static double HEADING_I = 0.0000000003; //0.0000000003
    public static double HEADING_D = 0.015; //0.02

    public static double XTranslational_P = 0.0; //0.025
    public static double XTranslational_I = 0; //0.0000000003
    public static double XTranslational_D = 0.00;
    public static double XTranslational_Ks = 0.0;//0.02

    public static double YTranslational_P = 0.0; //0.025
    public static double YTranslational_I = 0.0; //0.0000000003
    public static double YTranslational_D = 0.00; //0.02
    public static double YTranslational_Ks = 0.0;//0.02

    /*public static double XTranslational_P = 0.012; //0.025
    public static double XTranslational_I = 0; //0.0000000003
    public static double XTranslational_D = 0.001;
    public static double XTranslational_Ks = 0.09;//0.02

    public static double YTranslational_P = 0.012; //0.025
    public static double YTranslational_I = 0.0; //0.0000000003
    public static double YTranslational_D = 0.001; //0.02
    public static double YTranslational_Ks = 0.09;//0.02*/

}
