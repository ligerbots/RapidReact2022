package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    
    SparkMax m_intakeMotor;

    public Intake(){
        m_intakeMotor = new SparkMax(Constants.INTAKE_MOTOR_CAN_ID, MotorType.kBrushless);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void run(double speed) {
        m_intakeMotor.set(-speed);
    }
}
