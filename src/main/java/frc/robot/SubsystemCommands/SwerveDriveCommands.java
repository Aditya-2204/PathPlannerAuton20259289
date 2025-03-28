// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.SubsystemCommands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.commands.CommandConstants;
import frc.robot.subsystems.DrivetrainOld;

public class SwerveDriveCommands extends Command {
  private DrivetrainOld drivetrain = DrivetrainOld.getInstance();
  private double drive;
  private double turn;
  private double rotate;

  /** Creates a new SwerveDrive. */
  public SwerveDriveCommands(double drive, double turn, double rotate) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    this.turn = turn;
    this.rotate = rotate;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.swerveDrive(
        drive, 
        turn, 
        rotate,
        true,
        new Translation2d(),
        true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}