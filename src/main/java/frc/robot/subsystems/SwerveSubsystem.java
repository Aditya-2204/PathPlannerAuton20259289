// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModuleOld[] modules;
    private final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometry;
    private final Pigeon2 gyro;
    private final Field2d field = new Field2d();

    public SwerveSubsystem() {
        gyro = new Pigeon2(Constants.SwerveConstants.PIGEON_ID);
        modules = new SwerveModuleOld[]{
          
          Constants.SwerveConstants.leftFrontSwerveModule,
          Constants.SwerveConstants.rightFrontSwerveModule,
          Constants.SwerveConstants.leftBackSwerveModule,
          Constants.SwerveConstants.rightBackSwerveModule

        };
        
        kinematics = new SwerveDriveKinematics(

          Constants.SwerveConstants.flModuleOffset,
          Constants.SwerveConstants.frModuleOffset,
          Constants.SwerveConstants.blModuleOffset,
          Constants.SwerveConstants.brModuleOffset

        );

        odometry = new SwerveDriveOdometry(kinematics, getGyroRotation(), getPositions());

        try {
            RobotConfig config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                this::getPose,
                this::resetPose,
                this::getSpeeds,
                this::driveRobotRelative,
                new PPHolonomicDriveController(
                    Constants.SwerveConstants.translationConstants,
                    Constants.SwerveConstants.rotationConstants
                ),
                config,
                () -> DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red,
                this
            );
        } catch (Exception e) {
            DriverStation.reportError("Failed to load PathPlanner config", e.getStackTrace());
        }

        PathPlannerLogging.setLogActivePathCallback(poses -> field.getObject("path").setPoses(poses));
        SmartDashboard.putData("Field", field);
    }

    @Override
    public void periodic() {
        odometry.update(getGyroRotation(), getPositions());
        field.setRobotPose(getPose());
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void resetPose(Pose2d pose) {
        odometry.resetPosition(getGyroRotation(), getPositions(), pose);
    }

    public ChassisSpeeds getSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
        driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getPose().getRotation()));
    }

    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(robotRelativeSpeeds);
        setStates(targetStates);
    }

    public void setStates(SwerveModuleState[] targetStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, Constants.SwerveConstants.maxModuleSpeed);
        for (int i = 0; i < modules.length; i++) {
            modules[i].setDesiredState(targetStates[i]);
        }
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[modules.length];
        for (int i = 0; i < modules.length; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    public SwerveModulePosition[] getPositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
        for (int i = 0; i < modules.length; i++) {
            positions[i] = modules[i].getPosition();
        }
        return positions;
    }

    private Rotation2d getGyroRotation() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
    }
}
