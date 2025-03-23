package frc.robot.AutonCommands;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.SubsystemCommands.ArmCommands;
import frc.robot.SubsystemCommands.RollerCommands;
import frc.robot.SubsystemCommands.SwerveDriveCommands;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Roller;
import frc.robot.subsystems.DrivetrainOld;

import com.pathplanner.lib.commands.PathPlannerAuto;


public class LeftStartAutonwithPPLib extends SequentialCommandGroup{

    final double shooterSpeed = 1;

    public LeftStartAutonwithPPLib (Roller roller, Arm arm, DrivetrainOld swerveSubsystem)
    {
        addCommands(
            new PathPlannerAuto("LeftStartAuton"), //Left Start Auton
            new ArmCommands(-.60, arm).withTimeout(.25),
            new RollerCommands(-.3, roller).withTimeout(3),
            new RollerCommands(-0.2, roller).withTimeout(5)
        );
        // shoot, go straight, turn right, then go straight
    }
}