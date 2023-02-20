package frc.robot.graphics.src;

import java.awt.*;

import javax.swing.*;

public class NumVar extends JLabel
{
    private String variable_name;

    public NumVar(int x, int y, String name)
    {
        variable_name = name;
        this.setText(name + ": 0");
        this.setBounds(x, y, 150, 15);
        this.setForeground(Color.white);
        //this.setBackground(Color.white);
    }

    public void change_num(int num)
    {
        this.setText(variable_name + ": " + num);
    }

    public void change_position(int x, int y)
    {
        this.setBounds(x, y, 150, 15);
    }
}
