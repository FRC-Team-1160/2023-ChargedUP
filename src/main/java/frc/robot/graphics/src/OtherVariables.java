package frc.robot.graphics.src;

import java.awt.*;

import javax.swing.*;

public class OtherVariables extends JPanel{
    public NumVar variable1 = new NumVar(10, 30, "Variable 1");
    public NumVar variable2 = new NumVar(10, 70, "Variable 2");
    public NumVar variable3 = new NumVar(10, 110, "Variable 3");
    JLabel title = new JLabel("Other Variables");
    
    public OtherVariables()
    {
        title.setBounds(100, 5, 150, 15);
        title.setForeground(Color.white);
        title.setFont(new Font("Arial", Font.BOLD, 15));
        
        this.setBackground(new Color(81, 82, 81));
        this.setBounds(10, 10, 400, 140);
        this.add(title);
        this.add(variable1);
        this.add(variable2);
        this.add(variable3);
    }
}
