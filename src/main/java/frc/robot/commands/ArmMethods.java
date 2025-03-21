package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SubsystemCommands.ArmCommands;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Roller;

public class ArmMethods extends Command {
    Arm arm;
    Roller roller;
    Joystick button;

    public ArmMethods(Arm arm, Joystick button) {
        this.arm = arm;
        this.button = button;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        if (button.getRawButton(CommandConstants.ButtonShoulderL)) {
            ArmCommands cmd = new ArmCommands(.1, arm);
            //RollerCommands rcmd = new RollerCommands(-.3, roller);
            cmd.execute();
            //rcmd.execute();
        } else if (button.getRawButton(CommandConstants.ButtonShoulderR)) {
            ArmCommands cmd = new ArmCommands(-.1, arm);
            cmd.execute();
        } else if (button.getRawButton(CommandConstants.PovDown)) {
            ArmCommands cmd = new ArmCommands(.1, arm);
            cmd.execute45();
        } else {
            ArmCommands cmd = new ArmCommands(-.02, arm);
            cmd.execute();
        }
    }
}