package frc.robot.SubsystemCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class ArmCommands extends Command {
    Arm arm;
    double power;

    public ArmCommands(double speed, Arm arm) {
        this.arm = arm;
        this.power = speed;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        arm.move(power);
    }
    public void execute45() {
        ArmCommands move = new ArmCommands(power, arm);
        move.execute();
        Timer.delay(1);
        move.stop();
    }

    public void stop() {
        arm.move(0);
    } 
}
// Written by Wolfram121