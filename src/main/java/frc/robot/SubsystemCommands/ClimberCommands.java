package frc.robot.SubsystemCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class ClimberCommands extends Command {
    //Define Motor Attributes
    Climber climberMotors;
    double power;
    String motor;

   public ClimberCommands(Climber _climberMotors, double speed, String motor)
   {
        this.climberMotors = _climberMotors;
        this.power = speed;
        this.motor = motor; //This is the motor that we are going to be controlling: Options: Hand, Shoulder, Elbow, Wrist
        addRequirements(climberMotors);
   } 

   @Override
   public void execute()
   {
         //Set the speed of the motor
         switch(motor)
         {
              case "Hand":
                climberMotors.move_hand(power);
                break;
              case "Shoulder":
                climberMotors.move_shoulder(power);
                break;
              case "Elbow":
                climberMotors.move_elbow(power);
                break;
              case "Wrist":
                climberMotors.move_wrist(power);
                break;
         }
   }
}

//Created by Aditya-2204