package frc.robot.graphics.src;

import java.awt.*;

import javax.swing.*;

public class StringVar extends JLabel{
    private String variable_name;

    public StringVar(int x, int y, String name)
    {
        variable_name = name;
        this.setText(name + ": ");
        this.setBounds(x, y, 150, 15);
        this.setForeground(Color.white);
        //this.setBackground(Color.white);
    }

    public void change_string(String s)
    {
        this.setText(variable_name + ": " + s);
    }

    public void change_position(int x, int y)
    {
        this.setBounds(x, y, 150, 15);
    }
}
