package frc.robot.graphics.src;

import java.awt.*;
import javax.swing.*;
import java.awt.image.*;

public class Robot extends JPanel{
    private Image wheel_topLeft = new ImageIcon("src/main/java/frc/robot/graphics/images/RobotWheel.png").getImage();;
    private Image wheel_topRight = new ImageIcon("src/main/java/frc/robot/graphics/images/RobotWheel.png").getImage();;
    private Image wheel_bottomLeft = new ImageIcon("src/main/java/frc/robot/graphics/images/RobotWheel.png").getImage();;
    private Image wheel_bottomRight = new ImageIcon("src/main/java/frc/robot/graphics/images/RobotWheel.png").getImage();;
    private Image body = new ImageIcon("src/main/java/frc/robot/graphics/images/RobotBody.png").getImage();

    //top left wheel position
    private int x_tl = (int)(70 * Math.sin(Math.toRadians(225))) + 50;
    private int y_tl = (int)(70 * Math.cos(Math.toRadians(225))) + 50;

    //top right wheel position
    private int x_tr = (int)(70 * Math.sin(Math.toRadians(135))) + 55;
    private int y_tr = (int)(70 * Math.cos(Math.toRadians(135))) + 50;

    //bottom left wheel position
    private int x_bl = (int)(70 * Math.sin(Math.toRadians(315))) + 50;
    private int y_bl = (int)(70 * Math.cos(Math.toRadians(315))) + 50;

    //bottom right wheel position
    private int x_br = (int)(70 * Math.sin(Math.toRadians(45))) + 55;
    private int y_br = (int)(70 * Math.cos(Math.toRadians(45))) + 50;

    private int body_ang = 0;
    private int tl_ang = 0;
    private int tr_ang = 0;
    private int bl_ang = 0;
    private int br_ang = 0;

    private Graphics2D g2D;

    public Robot()
    {
        this.setBounds(640, 10, 150, 150);
        //this.setBackground(new Color(158, 187, 232));
    }

    public void paint(Graphics g)
    {
        super.paint(g);
        g2D = (Graphics2D) g;
        
        g2D.drawImage(body, 0, 0, null);
        g2D.drawImage(wheel_topLeft, x_tl, y_tl, null);
        g2D.drawImage(wheel_topRight, x_tr, y_tr, null);
        g2D.drawImage(wheel_bottomLeft, x_bl, y_bl, null);
        g2D.drawImage(wheel_bottomRight, x_br, y_br, null);
    }

    public void change_wheel_rotation(int new_ang, String pos) {
        this.rotateImage(new_ang, this, pos, "src/main/java/frc/robot/graphics/images/RobotWheel.png");
        repaint();
    }

    public void change_body_rotation(int new_ang) {
        if(body_ang != new_ang)
        {
            x_tl = (int)(70 * Math.sin(Math.toRadians(225 + new_ang))) + 50;
            y_tl = (int)(70 * Math.cos(Math.toRadians(225 + new_ang))) + 50;

            x_tr = (int)(70 * Math.sin(Math.toRadians(135 + new_ang))) + 55;
            y_tr = (int)(70 * Math.cos(Math.toRadians(135 + new_ang))) + 50;

            x_bl = (int)(70 * Math.sin(Math.toRadians(315 + new_ang))) + 50;
            y_bl = (int)(70 * Math.cos(Math.toRadians(315 + new_ang))) + 50;

            x_br = (int)(70 * Math.sin(Math.toRadians(45 + new_ang))) + 55;
            y_br = (int)(70 * Math.cos(Math.toRadians(45 + new_ang))) + 50;

            this.rotateWheel(tl_ang + new_ang, this, "Top-Left");
            this.rotateWheel(tr_ang + new_ang, this, "Top-Right");
            this.rotateWheel(bl_ang + new_ang, this, "Bottom-Left");
            this.rotateWheel(br_ang + new_ang, this, "Bottom-Right");

            System.out.println("x: " + x_tl + "y: " + y_tl);
            body_ang = new_ang;
        }
        this.rotateImage(new_ang, this, "Body", "src/main/java/frc/robot/graphics/images/RobotBody.png");
        repaint();
    }

    public void rotateImage(double degrees, ImageObserver o, String pos, String image)
    {
        ImageIcon icon = new ImageIcon(image);
        Image icon_img = new ImageIcon(image).getImage();
        BufferedImage blankCanvas = new BufferedImage(icon.getIconWidth(), icon.getIconHeight(), BufferedImage.TYPE_INT_ARGB);
        Graphics2D gg2d = (Graphics2D)blankCanvas.getGraphics();
        gg2d.rotate(Math.toRadians(-degrees), icon.getIconWidth()/2, icon.getIconHeight()/2);
        gg2d.drawImage(icon_img, 0, 0, o);
        
        if(pos.equals("Top-Left"))
        {
            this.wheel_topLeft = blankCanvas;
            tl_ang = (int) degrees;
        }
        else if(pos.equals("Top-Right"))
        {
            this.wheel_topRight = blankCanvas;
            tr_ang = (int) degrees;
        }
        else if(pos.equals("Bottom-Left"))
        {
            this.wheel_bottomLeft = blankCanvas;
            bl_ang = (int) degrees;
        }
        else if(pos.equals("Bottom-Right"))
        {
            this.wheel_bottomRight = blankCanvas;
            br_ang = (int) degrees;
        }
        else if(pos.equals("Body"))
        {
            this.body = blankCanvas;
        }
    }

    public void rotateWheel(double degrees, ImageObserver o, String pos)
    {
        ImageIcon icon = new ImageIcon("src/main/java/frc/robot/graphics/images/RobotWheel.png");
        Image icon_img = new ImageIcon("src/main/java/frc/robot/graphics/images/RobotWheel.png").getImage();
        BufferedImage blankCanvas = new BufferedImage(icon.getIconWidth(), icon.getIconHeight(), BufferedImage.TYPE_INT_ARGB);
        Graphics2D gg2d = (Graphics2D)blankCanvas.getGraphics();
        gg2d.rotate(Math.toRadians(-degrees), icon.getIconWidth()/2, icon.getIconHeight()/2);
        gg2d.drawImage(icon_img, 0, 0, o);
        
        if(pos.equals("Top-Left"))
        {
            this.wheel_topLeft = blankCanvas;
        }
        else if(pos.equals("Top-Right"))
        {
            this.wheel_topRight = blankCanvas;
        }
        else if(pos.equals("Bottom-Left"))
        {
            this.wheel_bottomLeft = blankCanvas;
        }
        else if(pos.equals("Bottom-Right"))
        {
            this.wheel_bottomRight = blankCanvas;
        }
    }
}
