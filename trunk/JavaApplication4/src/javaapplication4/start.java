/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package javaapplication4;
import javax.swing.*  ;
import java.awt.*      ;
import java.awt.event.* ;

/**
 *
 * @author Administrador
 */

import java.io.IOException;
import javax.imageio.ImageIO;
import java.io.File;

public class start {
    
    JFrame frame;
    ShapePanel panel;
    public void open()
    {
              // TODO code application logic here
           frame = new JFrame("JFrame");

           panel = new ShapePanel();

            frame.setSize(256,256);

            frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

            frame.getContentPane().add(panel);

            frame.pack();

            frame.show();
    }
    public void setImage(String s)
    {
        System.out.println("hola desde java: "+ s);

        try{
            panel.fondo = ImageIO.read(new File(s));
            panel.label2.setIcon(new ImageIcon(panel.fondo));
        }catch(IOException e)
        {
            System.out.println(e.getMessage());
        }
            
        
    }

}
