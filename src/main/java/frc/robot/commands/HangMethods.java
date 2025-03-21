package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hang;

import frc.robot.SubsystemCommands.HangCommands;


public class HangMethods extends Command {
    //Define attributes
    Hang _hanger;
    Joystick _joystick;

    public HangMethods(Hang _hanger, Joystick _controller) {
        //Construct Attributes
        this._hanger = _hanger;
        _joystick = _controller;
        addRequirements(_hanger);
    }

    @Override
    public void execute()
    {
        //Set speeds and directions based on button press
        if(_joystick.getRawButton(CommandConstants.ButtonX))
        {
            HangCommands _command = new HangCommands(_hanger, -.7);
            _command.execute();
        }
        else if(_joystick.getRawButton(CommandConstants.ButtonY))
        {
            HangCommands _command = new HangCommands(_hanger, .2);
            _command.execute();
        }
        else
        {
            HangCommands _command = new HangCommands(_hanger, 0);
            _command.execute();
        }
    }
}