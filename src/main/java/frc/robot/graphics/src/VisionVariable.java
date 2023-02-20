package frc.robot.graphics.src;

import java.awt.*;

import javax.swing.*;

public class VisionVariable extends JPanel{
    public NumVar variable1 = new NumVar(10, 30, "Variable 1");
    public NumVar variable2 = new NumVar(10, 70, "Variable 2");
    JLabel title = new JLabel("Vision Variables");

    public VisionVariable()
    {
        title.setBounds(220, 5, 150, 15);
        title.setForeground(Color.white);
        title.setFont(new Font("Arial", Font.BOLD, 15));
        
        this.setBackground(new Color(81, 82, 81));
        this.setBounds(10, 160, 600, 110);
        this.add(title);
        this.add(variable1);
        this.add(variable2);
    }
}
