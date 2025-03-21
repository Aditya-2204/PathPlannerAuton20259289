package frc.robot.AutonCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.SubsystemCommands.ArmCommands;
import frc.robot.SubsystemCommands.RollerCommands;
import frc.robot.SubsystemCommands.SwerveDriveCommands;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Roller;

public class MiddleStartAuto2 extends SequentialCommandGroup{
    final double shooterSpeed = 1;

    public MiddleStartAuto2(Roller roller, Arm arm)
    {
        addCommands(
            new SwerveDriveCommands(0,0,-0.06847999998).withTimeout(0.95),
            new SwerveDriveCommands(0.2588190451, 0.9659258263, 0).withTimeout(1.05762659673),
            new SwerveDriveCommands(0,0,0).withTimeout(1),
            new ArmCommands(shooterSpeed, arm).withTimeout(3),
            new RollerCommands(-.2, roller).withTimeout(5),
            new SwerveDriveCommands(0,0,0.4568191267).withTimeout(1.5),
            new SwerveDriveCommands(0.2679970506,0.9634197324,0).withTimeout(0.6179391352),
            new ArmCommands(-.15, arm).withTimeout(1),
            new RollerCommands(.3, roller).withTimeout(3)
        );
        //shoot, then either go straight or vee]=r slightly right then left
    }
}

//Made by Aditya-2204