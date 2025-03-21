package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorMiscConstants;


//Imma treat Climber like an human arm

public class Climber extends SubsystemBase {
    public SparkMax climberMotorShoulder;
    public SparkMax climberMotorElbow;
    public SparkMax climberMotorWrist;
    public SparkMax climberMotorHand;

    public Climber() {

        climberMotorShoulder = new SparkMax(MotorMiscConstants.SHOULDER_CLIMBER_ID, MotorType.kBrushless);
        climberMotorElbow = new SparkMax(MotorMiscConstants.ELBOW_CLIMBER_ID, MotorType.kBrushless);
        climberMotorHand = new SparkMax(MotorMiscConstants.HAND_CLIMBER_ID, MotorType.kBrushless);
        climberMotorWrist = new SparkMax(MotorMiscConstants.WRIST_CLIMBER_ID, MotorType.kBrushless);
    }

    public void move_shoulder(double speed){
        climberMotorShoulder.set(speed);
    }

    public void move_elbow(double speed){
        climberMotorElbow.set(speed);
    }

    public void move_wrist(double speed){
        climberMotorWrist.set(speed);
    }

    public void move_hand(double speed){
        climberMotorHand.set(speed);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climber Motor Shoulder "+MotorMiscConstants.SHOULDER_CLIMBER_ID, climberMotorShoulder.get());
        SmartDashboard.putNumber("Climber Motor Elbow "+MotorMiscConstants.ELBOW_CLIMBER_ID, climberMotorElbow.get());
        SmartDashboard.putNumber("Climber Motor Wrist "+MotorMiscConstants.HAND_CLIMBER_ID, climberMotorWrist.get());
        SmartDashboard.putNumber("Climber Motor Hand "+MotorMiscConstants.WRIST_CLIMBER_ID, climberMotorHand.get());
    }
}

//Created by Aditya-2204