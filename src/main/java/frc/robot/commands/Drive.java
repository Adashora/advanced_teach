// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Drive extends Command {


  Translation2d trans;
  Drivetrain dt;
  Joystick joystickR;
  Joystick joystickL;

  double dtx;
  double dty;

  SlewRateLimiter x_speed_limit = new SlewRateLimiter(3);
  SlewRateLimiter y_speed_limit = new SlewRateLimiter(3);
  SlewRateLimiter rot_speed_limit = new SlewRateLimiter(3);




  /** Creates a new Drive. */
  public Drive(Drivetrain dt, Joystick joystickR, Joystick joystickL) {


    this.dt = dt;
    this.joystickR = joystickR;
    this.joystickL = joystickL;
    addRequirements(this.dt);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    dtx = -x_speed_limit.calculate(MathUtil.applyDeadband(this.joystickL.getY(), 0.1));
    dty = -x_speed_limit.calculate(MathUtil.applyDeadband(this.joystickL.getX(), 0.1));
    double rot = rot_speed_limit.calculate(MathUtil.applyDeadband(this.joystickR.getX(), 0.1));


    trans = new Translation2d(dtx, dty).times(Constants.max_speed);

    this.dt.drive(trans, rot, true);





  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
