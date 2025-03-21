package frc.robot.subsystems;
//Aarav is doing all hang!

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.MotorMiscConstants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hang extends SubsystemBase {


    //Define Motor Constructor
    private SparkMax hangMotor12 = new SparkMax(MotorMiscConstants.HANG_MOTOR_ID, MotorType.kBrushless);

    public Hang(){
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
            
        hangMotor12.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    public void move(double speed){
        hangMotor12.set(speed);
        SmartDashboard.putNumber("Hang 12 power", hangMotor12.get());
    }
    public void stop(){
        //Stops Motor
        hangMotor12.set(0);
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Hang Motor 12", hangMotor12.get());
    }
}
