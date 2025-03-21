package frc.robot.AutonCommands;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.SubsystemCommands.ArmCommands;
import frc.robot.SubsystemCommands.RollerCommands;
import frc.robot.SubsystemCommands.SwerveDriveCommands;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Roller;
import frc.robot.subsystems.SwerveSubsystem;

import com.pathplanner.lib.commands.PathPlannerAuto;


public class RightStartAutonwithPPLib extends SequentialCommandGroup{

    final double shooterSpeed = 1;

    public RightStartAutonwithPPLib (Roller roller, Arm arm, SwerveSubsystem swerveSubsystem)
    {
        addCommands(
            new PathPlannerAuto("RightStartAuton"), //Left Start Auton
            new ArmCommands(-.60, arm).withTimeout(.25),
            new RollerCommands(-.3, roller).withTimeout(3),
            new RollerCommands(-0.2, roller).withTimeout(5)
        );
        // shoot, go straight, turn right, then go straight
    }
}