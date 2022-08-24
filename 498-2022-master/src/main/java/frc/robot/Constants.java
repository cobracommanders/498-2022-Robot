// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.util.TunableNumber;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final boolean tuningMode = false;
  public static final class DriveConstants{
    public static int kCountsPerRev = 2048;
    public static double kGearRatio = 7;//7:1
    public static double kWheelRadiusInches = 2.25;
    public static double maxSpeedMetersPerSec = 1.4;//2.67

    public static final double kTrackwidthMeters = Units.inchesToMeters(25.5);//~24.5in
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);//.609
    //test w/ sysid
    //public static double ksVolts = 0.7036;
    public static TunableNumber ksVolts = new TunableNumber("Drive/ks", .74895);//6
    //public static double kvVoltSecondsPerMeter = 1.5039;
    public static TunableNumber kvVoltSecondsPerMeter = new TunableNumber("Drive/kv", 2.3517);//2
    //public static double kaVoltSecondsSquaredPerMeter = 0.60267;
    public static TunableNumber kaVoltSecondsSquaredPerMeter = new TunableNumber("Drive/ka", 0.49078);//.60267//5
    public static TunableNumber kPDriveVel = new TunableNumber("Drive/kP", .911989);//3.3944);//23.515;////.6535;////2.0054;.911//.1635
    //public static TunableNumber kF = new TunableNumber("Drive/kF", 0);
    public static double kI = 0;
    public static double kD = 0.00;
    private static final double bumperlessRobotLength = Units.inchesToMeters(26);
    private static final double bumperlessRobotWidth = Units.inchesToMeters(24);
    private static final double bumperThickness = Units.inchesToMeters(3);

    public static final double fullRobotWidth = bumperlessRobotWidth + bumperThickness * 2;
    public static final double fullRobotLength = bumperlessRobotLength + bumperThickness * 2;
}
public final class AutoConstants{
    public static final double kMaxSpeedMetersPerSecond = 4;//2.75
    public static final double kMaxAccelerationMetersPerSecondSquared = 1.35;//1.6
    public static final double maxCentripetalAcceleration = 3.5;
    //good values for meters
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = .7; //.35
}
public final static class IntakeConstants{
  public final static TunableNumber kP = new TunableNumber("Intake/kP", .95);
  public final static TunableNumber kD = new TunableNumber("Intake/kD", .000057327);

  public final static TunableNumber kF = new TunableNumber("Intake/kF", 0);
  public final static double ks = 0.79888;
  public final static double kcos = -0.068745;
  public final static double kv = 0.70068;
  public final static double ka = 0.043048;

}
  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }
}//Climber CAN 14R, 15L
