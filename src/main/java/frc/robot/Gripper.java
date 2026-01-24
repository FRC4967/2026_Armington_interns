package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gripper extends SubsystemBase{
    private Solenoid gripSolenoid;
    private Solenoid releaseSolenoid;
    private PneumaticHub hub;
    private Compressor compressor;

    public Gripper(){
        hub = new PneumaticHub(8);
        gripSolenoid = hub.makeSolenoid(0);
        releaseSolenoid = hub.makeSolenoid(1);
        compressor = new Compressor(8, PneumaticsModuleType.REVPH);
        compressor.enableDigital();
    }

    public void grip(boolean gripIn){
        gripSolenoid.set(gripIn);
        releaseSolenoid.set(!gripIn);
    
    }
}
