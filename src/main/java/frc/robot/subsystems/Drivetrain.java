// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {



  public PigeonIMU gyro;

  public Swerve_module[] swerve_modules;




  public Drivetrain() {


    this.swerve_modules = new Swerve_module[] {


        new Swerve_module(Constants.dt.module_0.module_number, Constants.dt.module_0.dirve_id, Constants.dt.module_0.turn_id, Constants.dt.module_0.cancoder_id, Constants.dt.module_0.turn_offset),
        new Swerve_module(Constants.dt.module_1.module_number, Constants.dt.module_1.dirve_id, Constants.dt.module_1.turn_id, Constants.dt.module_1.cancoder_id, Constants.dt.module_1.turn_offset),
        new Swerve_module(Constants.dt.module_2.module_number, Constants.dt.module_2.dirve_id, Constants.dt.module_2.turn_id, Constants.dt.module_2.cancoder_id, Constants.dt.module_2.turn_offset),
        new Swerve_module(Constants.dt.module_3.module_number, Constants.dt.module_3.dirve_id, Constants.dt.module_3.turn_id, Constants.dt.module_3.cancoder_id, Constants.dt.module_3.turn_offset)

    };

      gyro = new PigeonIMU(10);

      gyro.configFactoryDefault();

      set_gyro(0);

    }

    public void set_gyro(double yaw){

      gyro.setYaw(yaw);
    }


    public Rotation2d get_Yaw(){

      return Rotation2d.fromDegrees(gyro.getYaw());


    }

    
public void drive(Translation2d translation, double rotation, boolean field_relative){

  SwerveModuleState[] states = Constants.swerve_map.toSwerveModuleStates(
    field_relative
     ? ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation, get_Yaw())
     : new ChassisSpeeds(translation.getX(), translation.getY(),rotation)
  );


  SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.max_speed);

  for (Swerve_module module : this.swerve_modules){


    module.set_desired_state(states[module.module_number]);
  }





}










  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}

