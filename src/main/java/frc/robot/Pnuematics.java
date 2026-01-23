package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Pnuematics {

    static Compressor compressor;

    public void Pnuematics(){
        compressor = new Compressor(8, PneumaticsModuleType.REVPH);
    }
    public static void start(){
        compressor.enableDigital();
    }
}
