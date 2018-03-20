package gui;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Container;
import java.awt.Dimension;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.KeyEvent;

import javax.swing.JPanel;
import javax.swing.JTextField;
import javax.swing.SpringLayout;

import internal.Testbed.Drone;
import internal.Testbed.World;
import internal.Testbed.WorldObject;
import math.Vector3f;

import javax.swing.AbstractButton;
import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JLabel;
 
public class TextWindow extends JPanel implements ActionListener{
	private static final long serialVersionUID = 1L;
	private static JFrame frame;
	static Container contentPane;
	protected JButton button1, button2, button3, button4;
	SpringLayout layout;
	private static Graphics graphics;
    
    public void update(World world) {
    	contentPane.removeAll();
    	addToContentPane(world);
    	addButtons();
    	contentPane.revalidate();
    	frame.repaint();
    }
    
    private void addToContentPane(World world) {
    	Vector3f velocity = new Vector3f();
    	Vector3f position = new Vector3f();
    	Vector3f orientation = new Vector3f();
    	float heading = 0;
    	float pitch = 0;
    	float roll = 0;
    	
    	
        for (WorldObject object: world.getObjectSet()) {
        	if (object.getClass() == Drone.class) {
        		velocity = ((Drone) object).getVelocity().convertToVector3f();
        		position = ((Drone) object).getPosition().convertToVector3f();
        		heading = ((Drone) object).getHeading();
        		pitch = ((Drone) object).getPitch();
        		roll = ((Drone) object).getRoll();

        	}
        }
    	
    	JLabel velocityLabel = new JLabel("Velocity: ");
    	JTextField velocityField = new JTextField(" ( " + velocity.x + ", " + velocity.y + ", " + velocity.z + " ) ");
    	velocityField.setEditable(false);
    	contentPane.add(velocityLabel);
    	contentPane.add(velocityField); 
    	
    	JLabel positionLabel = new JLabel("Position: ");
    	JTextField positionField = new JTextField(" ( " + position.x + ", " + position.y + ", " + position.z + " ) ");
    	positionField.setEditable(false);
    	contentPane.add(positionLabel);
    	contentPane.add(positionField); 
    	
    	JLabel headingLabel = new JLabel("Heading: ");
    	JTextField headingField = new JTextField(" ( " + heading + " ) ");
    	headingField.setEditable(false);
    	contentPane.add(headingLabel);
    	contentPane.add(headingField); 
    	
    	JLabel pitchLabel = new JLabel("Pitch: ");
    	JTextField pitchField = new JTextField(" ( " + pitch + " ) ");
    	pitchField.setEditable(false);
    	contentPane.add(pitchLabel);
    	contentPane.add(pitchField); 
    	
    	JLabel rollLabel = new JLabel("Roll: ");
    	JTextField rollField = new JTextField(" ( " + roll + " ) ");
    	rollField.setEditable(false);
    	contentPane.add(rollLabel);
    	contentPane.add(rollField); 
    	
    	layout = new SpringLayout();
        contentPane.setLayout(layout);
        
    	layout.putConstraint(SpringLayout.WEST, velocityLabel, 5, SpringLayout.WEST, contentPane);
    	layout.putConstraint(SpringLayout.NORTH, velocityLabel, 5, SpringLayout.NORTH, contentPane);
    	layout.putConstraint(SpringLayout.WEST, velocityField, 5, SpringLayout.EAST, velocityLabel);
    	layout.putConstraint(SpringLayout.NORTH, velocityField, 5, SpringLayout.NORTH, contentPane);
    	
    	layout.putConstraint(SpringLayout.WEST, positionLabel, 5, SpringLayout.WEST, contentPane);
    	layout.putConstraint(SpringLayout.NORTH, positionLabel, 30, SpringLayout.NORTH, contentPane);
    	layout.putConstraint(SpringLayout.WEST, positionField, 5, SpringLayout.EAST, positionLabel);
    	layout.putConstraint(SpringLayout.NORTH, positionField, 30, SpringLayout.NORTH, contentPane);
    	
    	layout.putConstraint(SpringLayout.WEST, headingLabel, 5, SpringLayout.WEST, contentPane);
    	layout.putConstraint(SpringLayout.NORTH, headingLabel, 55, SpringLayout.NORTH, contentPane);
    	layout.putConstraint(SpringLayout.WEST, headingField, 5, SpringLayout.EAST, headingLabel);
    	layout.putConstraint(SpringLayout.NORTH, headingField, 55, SpringLayout.NORTH, contentPane);
    	
    	layout.putConstraint(SpringLayout.WEST, pitchLabel, 5, SpringLayout.WEST, contentPane);
    	layout.putConstraint(SpringLayout.NORTH, pitchLabel, 80, SpringLayout.NORTH, contentPane);
    	layout.putConstraint(SpringLayout.WEST, pitchField, 5, SpringLayout.EAST, pitchLabel);
    	layout.putConstraint(SpringLayout.NORTH, pitchField, 80, SpringLayout.NORTH, contentPane);
    	
    	layout.putConstraint(SpringLayout.WEST, rollLabel, 5, SpringLayout.WEST, contentPane);
    	layout.putConstraint(SpringLayout.NORTH, rollLabel, 105, SpringLayout.NORTH, contentPane);
    	layout.putConstraint(SpringLayout.WEST, rollField, 5, SpringLayout.EAST, rollLabel);
    	layout.putConstraint(SpringLayout.NORTH, rollField, 105, SpringLayout.NORTH, contentPane);
    }
    
    public void initButtons() {
    	button1 = new JButton("drone view");
        button1.setVerticalTextPosition(AbstractButton.CENTER);
        button1.setHorizontalTextPosition(AbstractButton.LEADING);
        button1.setMnemonic(KeyEvent.VK_1);
        button1.setActionCommand("DRONE_CAM");
 
        button2 = new JButton("chase view");
        button2.setMnemonic(KeyEvent.VK_2);
        button2.setActionCommand("DRONE_CHASE_CAM");
        
        button3 = new JButton("top down view");
        button3.setMnemonic(KeyEvent.VK_3);
        button3.setActionCommand("DRONE_TOP_DOWN_CAM");
        
        button4 = new JButton("side view");
        button4.setMnemonic(KeyEvent.VK_4);
        button4.setActionCommand("DRONE_SIDE_CAM");
 
        button1.setBackground(Color.CYAN);
        button2.setBackground(Color.lightGray);
        button3.setBackground(Color.lightGray);
        button4.setBackground(Color.lightGray);
 
        button1.setToolTipText("Shows the drone from its perspective.");
        button2.setToolTipText("Shows the drone from behind.");
        button3.setToolTipText("Shows the drone from the top.");
        button4.setToolTipText("Shows the drone from the side.");
        
        button1.addActionListener(this);
        button2.addActionListener(this);
        button3.addActionListener(this);
        button4.addActionListener(this);
 
        addButtons();
    }
    
    public void addButtons() {
    	 
    	contentPane.add(button1);
        contentPane.add(button2);
        contentPane.add(button3);
        contentPane.add(button4);
        
        layout.putConstraint(SpringLayout.WEST, button1, 10, SpringLayout.WEST, contentPane);
    	layout.putConstraint(SpringLayout.NORTH, button1, 130, SpringLayout.NORTH, contentPane);
    	
    	layout.putConstraint(SpringLayout.WEST, button2, 10, SpringLayout.EAST, button1);
    	layout.putConstraint(SpringLayout.NORTH, button2, 130, SpringLayout.NORTH, contentPane);
    	
    	layout.putConstraint(SpringLayout.WEST, button3, 10, SpringLayout.EAST, button2);
    	layout.putConstraint(SpringLayout.NORTH, button3, 130, SpringLayout.NORTH, contentPane);
    	
    	layout.putConstraint(SpringLayout.WEST, button4, 10, SpringLayout.EAST, button3);
    	layout.putConstraint(SpringLayout.NORTH, button4, 130, SpringLayout.NORTH, contentPane);
    }

    public static TextWindow createAndShowWindow(World world, Graphics graphics, String title, int xDimension, int yDimension, int xPos, int yPos) {
    	
    	TextWindow.graphics = graphics;
    	
        frame = new JFrame(title);
        frame.setPreferredSize(new Dimension(xDimension, yDimension));
        frame.setLocation(xPos, yPos);
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        
        contentPane = frame.getContentPane();
        TextWindow window = new TextWindow();
        window.addToContentPane(world);
        window.initButtons();
        frame.setContentPane(contentPane);
 
        frame.pack();
        frame.setVisible(true);
        
        return window;
    }

	@Override
	public void actionPerformed(ActionEvent e) {
		if ("DRONE_CAM".equals(e.getActionCommand())) {
    		button1.setBackground(Color.CYAN);
            button2.setBackground(Color.lightGray);
            button3.setBackground(Color.lightGray);
            button4.setBackground(Color.lightGray);
        	for (String key: graphics.windows.keySet()) {
    			Window window = graphics.windows.get(key);
    			window.setSetting(Settings.DRONE_CAM);
    			break;
        	}
        } else if ("DRONE_CHASE_CAM".equals(e.getActionCommand()))  {
        	button1.setBackground(Color.lightGray);
            button2.setBackground(Color.CYAN);
            button3.setBackground(Color.lightGray);
            button4.setBackground(Color.lightGray);
        	for (String key: graphics.windows.keySet()) {
    			Window window = graphics.windows.get(key);
    			window.setSetting(Settings.DRONE_CHASE_CAM);
    			break;
        	}
        } else if ("DRONE_TOP_DOWN_CAM".equals(e.getActionCommand()))  {
        	button1.setBackground(Color.lightGray);
            button2.setBackground(Color.lightGray);
            button3.setBackground(Color.CYAN);
            button4.setBackground(Color.lightGray);
        	for (String key: graphics.windows.keySet()) {
    			Window window = graphics.windows.get(key);
    			window.setSetting(Settings.DRONE_TOP_DOWN_CAM);
    			break;
        	}
        } else if ("DRONE_SIDE_CAM".equals(e.getActionCommand()))  {
        	button1.setBackground(Color.lightGray);
            button2.setBackground(Color.lightGray);
            button3.setBackground(Color.lightGray);
            button4.setBackground(Color.CYAN);
        	for (String key: graphics.windows.keySet()) {
    			Window window = graphics.windows.get(key);
    			window.setSetting(Settings.DRONE_SIDE_CAM);
    			break;
        	}
        }
	}
}

