package frc.robot.SubsystemCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hang;


public class HangCommands extends Command {
    //Define Motor Attributes
    Hang hangMotor;
    double power;

   public HangCommands(Hang _hangMotor, double speed)
   {
        this.hangMotor = _hangMotor;
        this.power = speed;
        addRequirements(hangMotor);
   } 

   @Override
   public void execute()
   {
    //Move the Motor based on power
     hangMotor.move(power);
   }
}