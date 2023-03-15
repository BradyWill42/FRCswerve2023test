package frc.robot.util;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {

    private String id;
    private NetworkTable kTable;
    private NetworkTableEntry tv, tx, ty, ta, ts, tl, tshort, tlong, thor, tvert, getpipe; 

    public Limelight(String id) {
        this.id = id;
        kTable = NetworkTableInstance.getDefault().getTable("limelight-" + this.id);
        tv = kTable.getEntry("tv");
        tx = kTable.getEntry("tx");
        ty = kTable.getEntry("ty");
        ta = kTable.getEntry("ta");
        ts = kTable.getEntry("ts");
        tl = kTable.getEntry("tl");
        tshort = kTable.getEntry("tshort");
        tlong = kTable.getEntry("tlong");
        thor = kTable.getEntry("thor");
        tvert = kTable.getEntry("tvert");
        getpipe = kTable.getEntry("getpipe"); 
    }

    public Limelight() {
        this.id = "";
        kTable = NetworkTableInstance.getDefault().getTable("limelight");
        tv = kTable.getEntry("tv");
        tx = kTable.getEntry("tx");
        ty = kTable.getEntry("ty");
        ta = kTable.getEntry("ta");
        ts = kTable.getEntry("ts");
        tl = kTable.getEntry("tl");
        tshort = kTable.getEntry("tshort");
        tlong = kTable.getEntry("tlong");
        thor = kTable.getEntry("thor");
        tvert = kTable.getEntry("tvert");
        getpipe = kTable.getEntry("getpipe"); 
    }

    public boolean getValidTarget() {
        if(tv.getDouble(0.0) == 1.0) {
            return true;
        }
        return false;
    }
    public double getX() {
        return tx.getDouble(0.0);
    }
    public double getY() {
        return ty.getDouble(0.0);
    }
    public double getArea() {
        return ta.getDouble(0.0);   
    }
    public double getSkew() {
        return ts.getDouble(0.0);
    }
    public double getLatency() {
        return tl.getDouble(0.0);
    }
    public double getShort() {
        return tshort.getDouble(0.0);
    }
    public double getLong() {
        return tlong.getDouble(0.0);
    }
    public double getHorizontal() {
        return thor.getDouble(0.0);
    }
    public double getVertical() {
        return tvert.getDouble(0.0);
    }
    public double getPipeline() {
        return getpipe.getDouble(0.0);
    }
}