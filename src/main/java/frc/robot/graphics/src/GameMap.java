package frc.robot.graphics.src;

import java.awt.*;
import java.awt.event.*;

import javax.swing.*;

public class GameMap extends JPanel implements KeyListener{
    private Image minimize_robot = new ImageIcon("src/main/java/frc/robot/graphics/images/MinimizeRobot.png").getImage();
    private Image game_map = new ImageIcon("src/main/java/frc/robot/graphics/images/GameMap.png").getImage();
    private int x = 0;
    private int y = 0;

    public GameMap()
    {
        this.setBounds(650, 280, 600, 291);
        this.setBackground(new Color(81, 82, 81));
    }

    public void paint(Graphics g)
    {
        super.paint(g);
        Graphics2D g2D = (Graphics2D) g;
        
        g2D.drawImage(game_map, 0, 0, null);
        g2D.drawImage(minimize_robot, x, y, null);
    }

    public void changePositionX(int x_pos)
    {
        x = x + x_pos;

        if(x > 600 - 26)
        {
            x = 600 - 26;
        }
        else if(x < 0)
        {
            x = 0;
        }
        
        System.out.println("x: " + x);
        repaint();
    }

    public void changePositionY(int y_pos)
    {
        y = y + y_pos;

        if(y > 291 - 26)
        {
            y = 291 - 26;
        }
        else if(y < 0)
        {
            y = 0;
        }
        
        System.out.println("y: " + y);
        repaint();
    }

    @Override
    public void keyTyped(KeyEvent e) {
        // TODO Auto-generated method stub
        System.out.println(e.getKeyChar());
        char key = e.getKeyChar();

        if(key == 'a')
        {
            changePositionX(-10);
        }
        else if(key == 'd')
        {
            changePositionX(10);
        }
        
        if(key == 'w')
        {
            changePositionY(-10);
        }
        else if(key == 's')
        {
            changePositionY(10);
        }
    }

    @Override
    public void keyPressed(KeyEvent e) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void keyReleased(KeyEvent e) {
        // TODO Auto-generated method stub
        
    }
}
