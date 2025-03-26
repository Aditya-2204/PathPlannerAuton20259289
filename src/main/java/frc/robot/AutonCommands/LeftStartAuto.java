package frc.robot.AutonCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.SubsystemCommands.RollerCommands;
import frc.robot.SubsystemCommands.SwerveDriveCommands;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Roller;

public class LeftStartAuto extends SequentialCommandGroup{

    final double shooterSpeed = 1;

    public LeftStartAuto(Roller roller, Arm arm)
    {
        addCommands(
            new SwerveDriveCommands(0.2,0,0).withTimeout(0.89),/*Original:0.89 */
            new SwerveDriveCommands(0, 0, 0.3).withTimeout(1.2),/*Original:1.2 */

            new SwerveDriveCommands(0.19, 0.2,0).withTimeout(2.3),/*Original:2.3 */
            new SwerveDriveCommands(0.19, 0.2,0).withTimeout(0.4),/*Original:0.4 */
            new SwerveDriveCommands(0,0, 0.2).withTimeout(0.4),/*Original:0.4 */
            new SwerveDriveCommands(0,0,0).withTimeout(1),
            new RollerCommands(-0.2, roller).withTimeout(5)
            
        );
//TODO, TEST ON THURDSAY --> CHANGED TO NOT HAVE DEADLINEPARALLEL
        //Aarav Arora
        
        /* 
        addCommands(
            new SwerveDriveCommands(0, 0, -.2).withTimeout(.72),
            new SwerveDriveCommands(-.2, 0, 0).withTimeout(6),
            new ArmCommands(-.05, arm).withTimeout(3),
            new RollerCommands(-.2, roller).withTimeout(5)
            
        );
        */
        // shoot, go straight, turn right, then go straight
    }
}