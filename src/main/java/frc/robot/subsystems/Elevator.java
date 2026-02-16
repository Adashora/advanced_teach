// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {

  SparkMax l_motor;
  SparkMax r_motor;

  RelativeEncoder encoder;

  SparkMaxConfig r_motor_config;
  SparkMaxConfig l_motor_config;

  ProfiledPIDController pid;
  PIDController p_pid;

  SlewRateLimiter speed_limiter;


  /** Creates a new Elevator. */
  public Elevator() {

    r_motor = new SparkMax(Constants.ev.r_motor_ID, MotorType.kBrushless);
    l_motor = new SparkMax(Constants.ev.l_motor_ID, MotorType.kBrushless);

    encoder = r_motor.getEncoder();

    r_motor_config = new SparkMaxConfig();
    l_motor_config = new SparkMaxConfig();

    r_motor_config
    .idleMode(IdleMode.kBrake)
    .voltageCompensation(12)
    .inverted(false);

    r_motor_config.encoder
    .positionConversionFactor(Constants.ev.position_conversion_factor)
    .velocityConversionFactor(Constants.ev.velocity_conversion_factor);

    l_motor_config
    .idleMode(IdleMode.kBrake)
    .voltageCompensation(12)
    .inverted(false);

    l_motor_config.encoder
    .positionConversionFactor(Constants.ev.position_conversion_factor)
    .velocityConversionFactor(Constants.ev.velocity_conversion_factor);



    r_motor.configure(r_motor_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    l_motor.configure(l_motor_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);



    pid = new ProfiledPIDController(Constants.ev.kp, Constants.ev.ki, Constants.ev.kd, new TrapezoidProfile.Constraints(4, 4));
    p_pid = new PIDController(Constants.ev.kp, Constants.ev.ki, Constants.ev.kd);

    speed_limiter = new SlewRateLimiter(4.5);




  }

public double get_position() {

  return encoder.getPosition();
}

public double get_velocity() {

  return encoder.getVelocity();
}


public void reset_elevator() {

  encoder.setPosition(0);

}

public void run (double speed){

  double speed1 = speed_limiter.calculate(speed);

  r_motor.set(speed + Constants.ev.feed_forward);
  l_motor.set(speed + Constants.ev.feed_forward);

}


public void ev_to_roof(double position){

  

  double speed = MathUtil.clamp(p_pid.calculate(get_position(), position), -0.5, 0.5);

  run(speed);
}




  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
