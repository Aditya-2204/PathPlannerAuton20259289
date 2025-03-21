package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class ClimberMethods extends Command {
    // Define attributes
    Climber _climber;
    Joystick _joystick;

    public ClimberMethods(Climber _climber, Joystick _controller) {
        // Construct Attributes
        this._climber = _climber;
        _joystick = _controller;
        addRequirements(_climber);
    }

    @Override
    public void execute() {
        // Using joystick axes from CommandConstants for control
        double shoulderSpeed = _joystick.getRawAxis(CommandConstants.AxisLeftStickY); // Y-axis for shoulder control
        double elbowSpeed = _joystick.getRawAxis(CommandConstants.AxisLeftTrigger); // Left trigger for elbow control
        double wristSpeed = _joystick.getRawAxis(CommandConstants.AxisRightTrigger); // Right trigger for wrist control
        double handSpeed = _joystick.getRawAxis(CommandConstants.AxisRightStickX); // Right stick X-axis for hand control

        _climber.move_shoulder(shoulderSpeed * 0.5);
        _climber.move_elbow(elbowSpeed * 0.5);
        _climber.move_wrist(wristSpeed * 0.5);
        _climber.move_hand(handSpeed * 0.5);

        // Using buttons for specific actions
        if (_joystick.getRawButton(CommandConstants.ButtonA)) {
            _climber.move_hand(0.5); // Button A moves hand forward
        } else if (_joystick.getRawButton(CommandConstants.ButtonB)) {
            _climber.move_hand(-0.5); // Button B moves hand backward
        }
    }
}

//Created by Aditya-2204