// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {


  public static final double robot_length = Units.inchesToMeters(25);
  public static final double robot_width = Units.inchesToMeters(25);

  public static final Translation2d front_right = new Translation2d(robot_length/2, -robot_width/2);
  public static final Translation2d front_left = new Translation2d(robot_length/2, robot_width/2);
  public static final Translation2d back_right = new Translation2d(robot_length/2, -robot_width/2);
  public static final Translation2d back_left = new Translation2d(robot_length/2, robot_width/2);


  public static final SwerveDriveKinematics swerve_map = new SwerveDriveKinematics(
  front_left,
  front_right,
  back_right,
  back_left);



  public static final int max_speed = 5;

  public static final double drive_kp = 1;
  public static final double drive_ki = 0;
  public static final double drive_kd = 0;

  public static final double turn_kp = 1;
  public static final double turn_ki = 0;
  public static final double turn_kd = 0;


  public static final double encoder_tick_ratio = 42;
  public static final double drive_motor_ratio = 6.12;
  public static final double turn_motor_ratio = 150/7;


  public static final double position_conversion_factor = Units.feetToMeters(4*Math.PI /(encoder_tick_ratio * drive_motor_ratio));
  public static final double velocity_conversion_factor = position_conversion_factor /60.0;
  public static final double turn_position_conversion_factor = 360.0 / turn_motor_ratio;


  public class ev{

    public static final int r_motor_ID = 13;
    public static final int l_motor_ID = 14;

    public static final double position_conversion_factor = Units.inchesToMeters(35.625) / 39.1663;
    public static final double velocity_conversion_factor = Constants.position_conversion_factor / 60;


    public static final double kp = 0;
    public static final double ki = 0;
    public static final double kd = 0;

    public static final double speed = 0.5;

    public static final double feed_forward = 0.0;




  }



  public class dt{

    public class module_0{

      public static final int dirve_id = 1;
      public static final int turn_id = 2;
      public static final int cancoder_id = 3;
      public static Rotation2d turn_offset = Rotation2d.fromDegrees(0);
      public static final int module_number = 0;



    }
    public class module_1{

      public static final int dirve_id = 4;
      public static final int turn_id = 5;
      public static final int cancoder_id = 6;
      public static Rotation2d turn_offset = Rotation2d.fromDegrees(0);
      public static final int module_number = 1;


      
    }
    public class module_2{


      public static final int dirve_id = 7;
      public static final int turn_id = 8;
      public static final int cancoder_id = 9;
      public static Rotation2d turn_offset = Rotation2d.fromDegrees(0);
      public static final int module_number = 2;

      
    }
    public class module_3{

      public static final int dirve_id = 10;
      public static final int turn_id = 11;
      public static final int cancoder_id = 12;
      public static Rotation2d turn_offset = Rotation2d.fromDegrees(0);
      public static final int module_number = 3;


      
    }




  }



  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
