package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.subsystems.*;
import frc.robot.AutonCommands.*;
import frc.robot.commands.*;
import frc.robot.SubsystemCommands.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

public class RobotContainer {
  //Define Subsystems & Commands
  public static final Joystick driverController = new Joystick(0);
  public static final JoystickButton resetHeading_Start = new JoystickButton(driverController, CommandConstants.ButtonRightStick);
  private static Hang hang = new Hang();
  private static Arm arm = new Arm();
  private static Roller roller = new Roller();
  private final DrivetrainOld drivetrain = DrivetrainOld.getInstance();

  // Initializing commands to put up as choices
  private final Command leftCommand = new LeftStartAuto(roller, arm);
  private final Command middleCommand = new MiddleStartAuto(roller, arm);
  private final Command rightCommand = new RightStartAuto(roller, arm);
  private final Command nonSpeakerCommand = new NonSpeakerStartAuto();

  // Autons for PathPlanner
  private final Command leftPathPlannerCommand = new LeftStartAutonwithPPLib(roller, arm, drivetrain);
  private final Command middlePathPlannerCommand = new MiddleStartAutonwithPPLib(roller, arm, drivetrain);
  private final Command rightPathPlannerCommand = new RightStartAutonwithPPLib(roller, arm, drivetrain);

  SendableChooser<Command> m_chooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    drivetrain.setDefaultCommand(new SwerveDriveOld());
    configureBindings();
    m_chooser = new SendableChooser<>();

    // Set up choices for autonomous program
    m_chooser.addOption("Left Start", leftCommand);
    m_chooser.addOption("Middle Start", middleCommand);
    m_chooser.addOption("Right Start", rightCommand);

    // Autons for PathPlanner
    m_chooser.addOption("Left Start PathPlanner", leftPathPlannerCommand);
    m_chooser.addOption("Middle Start PathPlanner", middlePathPlannerCommand);
    m_chooser.addOption("Right Start PathPlanner", rightPathPlannerCommand);

    SmartDashboard.putData("Autonomous Chooser", m_chooser);
  }

  /**
   * Use this method to define your trigger->command mappings.
   */
  private void configureBindings() {
    // Configure the trigger bindings
    resetHeading_Start.onTrue(new InstantCommand(drivetrain::zeroHeading, drivetrain));
    hang.setDefaultCommand(new HangMethods(hang, driverController));
    arm.setDefaultCommand(new ArmMethods(arm, driverController));
    roller.setDefaultCommand(new RollerMethods(roller, driverController));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}
