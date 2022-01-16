package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    
    CANSparkMax intakeMotor;

    public Intake(){
        intakeMotor = new CANSparkMax(Constants.INTAKE_MOTOR_CAN_ID, MotorType.kBrushless);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void run(double speed) {
        intakeMotor.set(-speed);
    }

    public void IntakeCargo() {
        run(0.25);
    }

    public void OutputCargo() {
        run(-0.25);
    }
}
