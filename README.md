# Hawkbibi

### Subsystems
- Swerve drivetrain
- Elevator with preset levels
- Pneumatic claw
- LED control to indicate robot status

### Vision
- Uses limelight to detect cone nodes
- Aligns to nodes using PID loop

### Autonomous
4 red side autons (all place a high cone):
- Red 1: Place and mobility (used in front of driver station 1 on left-most cone node)
- Red 2: Place (can place high cube) and auto balance (used in front of driver station 2)
- Red 3: Place (can place high cube), mobility, and auto balance (used in front of driver station 2)
- Red 4: Place, move to center line (used in front of driver station 3 on right-most cone node)

Blue side autons are the same but red 1 and red 4 are swapped (i.e. blue 4 corresponds to red 1 and blue 1 corresponds to red 4)
Red 2 and red 3 are the same as blue 2 and blue 3 respectively

### Controls

#### Driver
- Left stick: Translation
- Right stick: Rotation
- Left trigger: Auto align to cone node (use while in slow mode)
- Right trigger: Slow mode toggle
- Left bumnper: While held robot-centric mode
- B: Zero gyro

#### Operator
- Buttons on button board
- Joystick to manually move elevator
