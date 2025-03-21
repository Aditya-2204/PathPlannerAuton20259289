package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants.MotorMiscConstants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Roller extends SubsystemBase {
    
    // Provide all required parameters
    private SparkMax roller = new SparkMax(MotorMiscConstants.ROLLER_MOTOR_ID, MotorType.kBrushless);

    public Roller() {
        SparkMaxConfig config = new SparkMaxConfig();
        config
            .inverted(true)
            .idleMode(IdleMode.kBrake);
        config.encoder
            .positionConversionFactor(1000)
            .velocityConversionFactor(1000);
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(1.0, 0.0, 0.0);
            
        roller.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
    }

    public void move(double speed) {
        roller.set(speed);
        SmartDashboard.putNumber("Roller Motor 10", roller.get());
    }

    public void stop() {
        roller.set(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Roller Motor 10", roller.get());
    }

}

/*
 * Created by Aditya-2204
 */