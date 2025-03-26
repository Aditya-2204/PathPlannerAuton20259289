// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.SubsystemCommands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.commands.CommandConstants;
import frc.robot.subsystems.DrivetrainOld;

public class SwerveDriveOld extends Command {
  private DrivetrainOld drivetrain = DrivetrainOld.getInstance();
  private boolean fieldOriented = true;

  /** Creates a new SwerveDrive. */
  public SwerveDriveOld() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double frontSpeedInput = RobotContainer.driverController.getRawAxis(CommandConstants.AxisLeftStickY);
    double sideSpeedInput = RobotContainer.driverController.getRawAxis(CommandConstants.AxisLeftStickX);
    double turnSpeedInput = RobotContainer.driverController.getRawAxis(CommandConstants.AxisRightStickX);

    //Square scaling
    double frontSpeedOutput = Math.copySign(frontSpeedInput * frontSpeedInput, frontSpeedInput);
    double sideSpeedOutput = Math.copySign(sideSpeedInput * sideSpeedInput, sideSpeedInput);
    double turnSpeedOutput = turnSpeedInput;


    drivetrain.swerveDrive(
        frontSpeedOutput,
        sideSpeedOutput,
        turnSpeedOutput,
        fieldOriented,
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