// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ev_to_position extends Command {

  Elevator elevator;
  double position;
  /** Creates a new ev_to_position. */
  public ev_to_position(Elevator elevator, double position) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.elevator = elevator;
    this.position = position;

    addRequirements(this.elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    this.elevator.ev_to_roof(this.position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    this.elevator.run(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if(this.elevator.get_position() > this.position - 0.05 && this.elevator.get_position() < this.position + 0.05) {
      return true;
    } else {
      return false;
    }


  }
}
