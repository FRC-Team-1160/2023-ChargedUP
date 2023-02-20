package frc.robot.graphics.src;
import java.awt.*;
import java.awt.event.*;

public class MyThread extends Driver implements Runnable
{
    Constructor con;
    public MyThread(Constructor construc)
    {
        con = construc;
    }

    @Override
    public void run()
    {
        while(true)
        {
            //mouseposition.change_num(MyFrame.mousePosX);
            //mouseposition.repaint();
            con.other_var.title.setLocation(130, 5);
            con.other_var.variable1.setLocation(10, 30);
            con.other_var.variable2.setLocation(10, 70);
            con.other_var.variable3.setLocation(10, 110);

            con.vis_var.title.setLocation(220, 5);
            con.vis_var.variable1.setLocation(10, 30);
            con.vis_var.variable2.setLocation(10, 70);

            con.swer_var.title.setLocation(35, 5);
            con.swer_var.variable1.setLocation(10, 30);
            con.swer_var.variable2.setLocation(10, 70);
            con.swer_var.variable3.setLocation(10, 110);
            
            con.other_var.variable1.change_num(con.frame.mousePosX);
            con.rob.change_wheel_rotation(con.frame.mousePosX, "Top-Left");
            con.rob.change_wheel_rotation(-con.frame.mousePosX, "Top-Right");
            con.rob.change_body_rotation(-con.frame.mousePosX);
        }
    }
}
