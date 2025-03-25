package frc.robot.AutonCommands;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.SubsystemCommands.ArmCommands;
import frc.robot.SubsystemCommands.RollerCommands;
import frc.robot.SubsystemCommands.SwerveDriveCommands;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Roller;
import frc.robot.subsystems.DrivetrainOld;

import com.pathplanner.lib.commands.PathPlannerAuto;


public class MiddleStartAutonwithPPLib extends SequentialCommandGroup{

    final double shooterSpeed = 1;

    public MiddleStartAutonwithPPLib (Roller roller, Arm arm, DrivetrainOld swerveSubsystem)
    {
        addCommands(
            new PathPlannerAuto("MiddleStartAuton") //Middle Start Auton
        );
        // shoot, go straight, turn right, then go straight
    }
}