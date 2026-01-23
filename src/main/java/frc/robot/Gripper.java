package frc.robot;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gripper extends SubsystemBase{
    private Solenoid gripSolenoid;
    private Solenoid releaseSolenoid;
    private PneumaticHub hub;
    public Pnuematics pnuematics = new Pnuematics();

    public Gripper(){
        hub = new PneumaticHub(8);
        gripSolenoid = hub.makeSolenoid(0);
        releaseSolenoid = hub.makeSolenoid(1);
    }

    public void grip(boolean gripIn){
        gripSolenoid.set(!gripIn);
        releaseSolenoid.set(gripIn);
    }
}
