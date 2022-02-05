package frc.robot;
import edu.wpi.first.networktables.*;

public class Limelight {
    NetworkTable camera;

public Limelight(NetworkTable cam)
{
    camera = cam;
}
///true if target visible, false otherwise
public boolean visible()
{
    double v = camera.getEntry("tv").getDouble(0);

    if(v == 0)
{
        return false;
    } else {
        return true;
    }

}
//self explanatory
public double angleOffsetX() {
    return camera.getEntry("tx").getDouble(0);
}

public double angleOffsetY() {
    return camera.getEntry("ty").getDouble(0);
}


//set the vision pipeline
public void setPipeline(int pipe)
{
    camera.getEntry("pipeline").setNumber(pipe);
}
}