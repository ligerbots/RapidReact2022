package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    
    CANSparkMax m_intakeMotor;

    public Intake(){
        m_intakeMotor = new CANSparkMax(Constants.INTAKE_MOTOR_CAN_ID, MotorType.kBrushless);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void run(double speed) {
        m_intakeMotor.set(-speed);
    }
}
