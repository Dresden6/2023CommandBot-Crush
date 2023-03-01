package frc.robot.subsystems;
import javax.lang.model.util.ElementScanner14;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class Arm {
    private CANSparkMax armMotor;

    armMotor = new CANSparkMax(7, MotorType.kBrushless);

    public Arm(boolean grabMode) {
        this.grabMode = grabMode;
    }

    private boolean grabMode = false;

    if(grabMode == true)
    {
        clawGrab();
    }
    else 
    {
        clawRelease();
    }

}

private void clawGrab(){
/*
 * run the grab instructions
 */
}

private void clawRelease(){
/*
 * run the release instructions
 */
}