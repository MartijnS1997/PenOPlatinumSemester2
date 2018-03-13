package gui;

import java.awt.BorderLayout;
import java.awt.Container;
import java.awt.Dimension;

import javax.swing.JPanel;
import javax.swing.JTextField;
import javax.swing.SpringLayout;

import internal.Testbed.Drone;
import internal.Testbed.World;
import internal.Testbed.WorldObject;
import math.Vector3f;

import javax.swing.JFrame;
import javax.swing.JLabel;
 
public class TextWindow extends JPanel{
	private static final long serialVersionUID = 1L;
	private static JFrame frame;
	static Container contentPane;
    
    public static void update(World world) {
    	contentPane.removeAll();
    	addToContentPane(world);
    	contentPane.revalidate();
    	frame.repaint();
    }
    
    private static void addToContentPane(World world) {
    	Vector3f velocity = new Vector3f();
    	
        for (WorldObject object: world.getObjectSet()) {
        	if (object.getClass() == Drone.class) {
        		velocity = ((Drone) object).getVelocity().convertToVector3f();
        	}
        }
        
    	
    	JLabel velocityLabel = new JLabel("Velocity: ");
    	JTextField velocityField = new JTextField(" ( " + velocity.x + ", " + velocity.y + ", " + velocity.z + " ) ");
    	velocityField.setEditable(false);
    	contentPane.add(velocityLabel);
    	contentPane.add(velocityField); 
    	
    	
    	SpringLayout layout = new SpringLayout();
        contentPane.setLayout(layout);
    	layout.putConstraint(SpringLayout.WEST, velocityLabel, 5, SpringLayout.WEST, contentPane);
    	layout.putConstraint(SpringLayout.NORTH, velocityLabel, 5, SpringLayout.NORTH, contentPane);
    	layout.putConstraint(SpringLayout.WEST, velocityField, 5, SpringLayout.EAST, velocityLabel);
    	layout.putConstraint(SpringLayout.NORTH, velocityField, 5, SpringLayout.NORTH, contentPane);
    }

    public static void createAndShowWindow(World world, Graphics graphics, String title, int xDimension, int yDimension) {
    	
        frame = new JFrame(title);
        frame.setPreferredSize(new Dimension(xDimension, yDimension));
        frame.setLocation(-10, 0);
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        
        contentPane = frame.getContentPane();
        addToContentPane(world);
        frame.setContentPane(contentPane);
 
        frame.pack();
        frame.setVisible(true);
    }
}

