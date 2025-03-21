package frc.robot.AutonCommands;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.SubsystemCommands.ArmCommands;
import frc.robot.SubsystemCommands.RollerCommands;
import frc.robot.SubsystemCommands.SwerveDriveCommands;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Roller;


public class LeftStartAuto2 extends SequentialCommandGroup{

    final double shooterSpeed = 1;

    public LeftStartAuto2 (Roller roller, Arm arm)
    {
        addCommands(
            new SwerveDriveCommands(0, 0, -.2).withTimeout(.72),
            new SwerveDriveCommands(-.6, 0, 0).withTimeout(2),
            new ArmCommands(-.60, arm).withTimeout(.25),
            new RollerCommands(-.3, roller).withTimeout(3),
            new ArmCommands(.15, arm).withTimeout(1),
            new SwerveDriveCommands(-1, 0, 0).withTimeout(0.5),
            new SwerveDriveCommands(0, 0, .2).withTimeout(.72),
            new SwerveDriveCommands(1, 0, 0).withTimeout(1)
        );
        // shoot, go straight, turn right, then go straight
    }
}
//Made by Wolfram121