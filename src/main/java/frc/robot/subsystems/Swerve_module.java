// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/** Add your docs here. */
public class Swerve_module {

    public SparkMax drive_motor;
    public SparkMax turn_motor;

    public double turn_speed;
    public double drive_speed;

    public CANcoder best_encoder;
    public RelativeEncoder drive_encoder;
    public RelativeEncoder turn_encoder;

    public int module_number;

    PIDController turn_PID;
    PIDController drive_PID;

    Rotation2d turn_offset;
    SparkMaxConfig drive_config;
    SparkMaxConfig turn_config;

    private MagnetSensorConfigs magnet_sensor_config;




    public Swerve_module(int module_number, int dirve_id, int turn_id, int cancoder_id, Rotation2d turn_offset){



        this.turn_offset = turn_offset;
        this.drive_config = new SparkMaxConfig(); 
        this.turn_config = new SparkMaxConfig();

        this.module_number = module_number; 

        this.best_encoder = new CANcoder(cancoder_id);

        this.drive_motor = new SparkMax(dirve_id, MotorType.kBrushless);
        this.drive_encoder = this.drive_motor.getEncoder();

        drive_config
            .voltageCompensation(12)
            .smartCurrentLimit(50)
            .idleMode(IdleMode.kBrake)
            .inverted(false);
        drive_config.encoder
        .positionConversionFactor(Constants.position_conversion_factor)
        .velocityConversionFactor(Constants.velocity_conversion_factor);

        this.drive_motor.configure(drive_config,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        

        drive_PID = new PIDController(Constants.drive_ki, Constants.drive_kp, Constants.drive_kd);

        this.turn_motor = new SparkMax(turn_id, MotorType.kBrushless);
        this.turn_encoder = this.turn_motor.getEncoder();

        turn_config
            .voltageCompensation(12)
            .smartCurrentLimit(20)
            .idleMode(IdleMode.kBrake)
            .inverted(false);

            turn_config.encoder
            .positionConversionFactor(Constants.turn_position_conversion_factor);
            
            turn_config.signals
            .primaryEncoderPositionPeriodMs(500);



            this.turn_motor.configure(turn_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

            turn_PID = new PIDController(Constants.turn_kp, Constants.turn_ki, Constants.turn_kd);

    }

    public Rotation2d getCANCoder(){


        return Rotation2d.fromDegrees(this.best_encoder.getAbsolutePosition().getValue().magnitude() * 360);
    }

    public SwerveModuleState get_State(){
        //double speed_ms = this.drive_encoder.getVelocity();
        //Rotation2d angle = getCANCoder();

        return new SwerveModuleState(this.drive_encoder.getVelocity(), getCANCoder());

    }

    public void set_desired_state(SwerveModuleState desired_State){

        SwerveModuleState current_state = this.get_State();

        Rotation2d current_rotation = current_state.angle.minus(turn_offset);



        desired_State.optimize(current_rotation);

        desired_State.cosineScale(current_rotation);


        Rotation2d desired_rotation = desired_State.angle;

        double diff = desired_rotation.getDegrees() - current_rotation.getDegrees();


        if (Math.abs(diff) < 1) {
            turn_speed = 0;
        } 
        else {
            turn_speed = turn_PID.calculate(diff, 0);
        }

        drive_speed = desired_State.speedMetersPerSecond / Constants.max_speed;

        turn_motor.set(turn_speed);
        drive_motor.set(drive_speed);

        SmartDashboard.putNumber("desired Rotation" + this.module_number, desired_rotation.getDegrees());
        SmartDashboard.putNumber("current Rotation" + this.module_number, current_rotation.getDegrees());



    }
    

}
