package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class Arm {
    private CANSparkMax armMotor;
    private boolean grabMode;


    public Arm(boolean grabMode) {
        
        armMotor = new CANSparkMax(7, MotorType.kBrushless);
        this.grabMode = grabMode;

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
}