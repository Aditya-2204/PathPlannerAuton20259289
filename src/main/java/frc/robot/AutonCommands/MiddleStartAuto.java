package frc.robot.AutonCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.SubsystemCommands.ArmCommands;
import frc.robot.SubsystemCommands.RollerCommands;
import frc.robot.SubsystemCommands.SwerveDriveCommands;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Roller;


public class MiddleStartAuto extends SequentialCommandGroup{
    final double shooterSpeed = 1;

    public MiddleStartAuto(Roller roller, Arm arm)
    {
        
        addCommands(
            new SwerveDriveCommands(0, 0, 0.3).withTimeout(1.465),/*Original:1.465 */
            new SwerveDriveCommands(0.2, 0, 0).withTimeout(4),/*Original:4 */
            //new SwerveDriveCommands(0, 0.2, 0).withTimeout(1),
            new SwerveDriveCommands(0,0,0).withTimeout(1),
            new RollerCommands(-.2, roller).withTimeout(5)
        );
        //Rotate, then drive forward, strafe, and outtake
        //Aarav Arora
    }
}
