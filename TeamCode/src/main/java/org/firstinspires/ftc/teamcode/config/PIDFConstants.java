package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;

@Config
@Configurable
public class PIDFConstants {
    public static double TOP_SHOOTER_P = 0.009;
    public static double TOP_SHOOTER_I = 0.0;
    public static double TOP_SHOOTER_D = 0.0;
    public static double TOP_SHOOTER_FF = 0.0008;

    public static double BOTTOM_SHOOTER_P = 0.004;
    public static double BOTTOM_SHOOTER_I = 0.0;
    public static double BOTTOM_SHOOTER_D = 0.001;
    public static double BOTTOM_SHOOTER_FF = 0.0005;

    public static double HEADING_P = 0.04; //0.025
    public static double HEADING_I = 0.0000000003; //0.0000000003
    public static double HEADING_D = 0.015; //0.02
}
