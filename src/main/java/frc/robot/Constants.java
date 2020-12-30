/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static int right_port_1 = 2;//encoder
    public static int right_port_2 = 1;
    public static int left_port_1 = 3;//encoder
    public static int left_port_2 = 4;
    public static int xbox_port = 5;
    public static double max_pwr = 0.7;
    public static double min_pwr = 0.1;
    public static double p1 = 0.3;
    public static int gyro_port = 6;
    public static double diameter = 0.1524;
    public static double perimiter = diameter*2*Math.PI;
    public static double pulses_in_spins = 800;
    public static double pulses_in_meter = pulses_in_spins/perimiter;
    public static double pulses_in_milisecond = pulses_in_spins/perimiter;
    public static double kv = 2.34;
    public static double ks = 0.797;
    public static double ka = 0.862;
    public static double kp = 0.016;
    public static double ki = 0;
    public static double kd = 0.0016;
    //public static double ff = kv + ks*speed +ka*a;
}
