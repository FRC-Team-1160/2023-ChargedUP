package frc.robot.graphics.src;

import java.awt.*;
import java.awt.event.MouseEvent;

import javax.swing.*;
import javax.swing.event.MouseInputListener;

public class MyFrame extends JFrame implements MouseInputListener{
    public int mousePosX = 0;
    public int mousePosY = 0;
    //private Image background_img = new ImageIcon("images/DashboardBackground.png").getImage();
    
    MyFrame()
    {
        this.setTitle("GUI Dashboard");
        this.setSize(1550, 850);
        this.setVisible(true); //make frame visible
        this.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        this.setResizable(false);
        this.setLayout(null);
        this.addMouseListener(this);
        this.getContentPane().setBackground(Color.black);

        this.setBackground(Color.black);
    }

    public void addLabel(Label label)
    {
        this.add(label);
    }

    /* 
    public void paint(Graphics g)
    {
        Graphics2D g2D = (Graphics2D) g;
        
        //g2D.rotate(Math.toRadians(ang), body_img_icon.getIconWidth() / 2, body_img_icon.getIconHeight() / 2);
        g2D.drawImage(background_img, 0, 0, null);
    }
    */

    @Override
    public void mouseClicked(MouseEvent e) {
        // TODO Auto-generated method stub
        mousePosX = e.getX();
        mousePosY = e.getY();
    }

    @Override
    public void mousePressed(MouseEvent e) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void mouseReleased(MouseEvent e) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void mouseEntered(MouseEvent e) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void mouseExited(MouseEvent e) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void mouseDragged(MouseEvent e) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void mouseMoved(MouseEvent e) {
        // TODO Auto-generated method stub
        
    }
}
