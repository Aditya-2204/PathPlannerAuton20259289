package frc.robot.SubsystemCommands;

import frc.robot.subsystems.Roller;
import edu.wpi.first.wpilibj2.command.Command;

public class RollerCommands extends Command{
    Roller roller;
    double power;
    
    public RollerCommands(double speed, Roller roller) {
        this.roller = roller;
        this.power = speed;
        addRequirements(roller);
    }

    public void execute() {
        roller.move(power);
    }

    public void stop() {
        roller.move(0);
    }
}