import rtde_control
import rtde_receive
import time
import json
import os

# Connect to the robot
robot_ip = "129.244.148.233"  # ursimr robot ip
#robot_ip = "192.168.5.4" real robot
rtde_c = rtde_control.RTDEControlInterface(robot_ip)
rtde_r = rtde_receive.RTDEReceiveInterface(robot_ip)

# File to save positions
positions_file = "robot_positions.json"

# Load existing positions if file exists
positions = {}
if os.path.exists(positions_file):
    with open(positions_file, 'r') as f:
        positions = json.load(f)
    print(f"Loaded {len(positions)} previously saved positions.")

print("\n==== Robot Position Capture Tool ====")
print("This tool will help you capture positions in freedrive mode")

# Function to enable freedrive mode
def enable_freedrive():
    print("\nEnabling freedrive mode. You can now move the robot freely.")
    print("Position the robot and press Enter to capture the position.")
    rtde_c.teachMode()
    
# Function to disable freedrive mode
def disable_freedrive():
    print("Disabling freedrive mode.")
    rtde_c.endTeachMode()

try:
    # Main capture loop
    while True:
        position_name = input("\nEnter name for this position (or 'exit' to quit): ")
        
        if position_name.lower() == 'exit':
            break
            
        # Enable freedrive
        #enable_freedrive()
        
        # Wait for user to position robot and press Enter
        input("Position the robot and press Enter to capture...")
        
        # Get current position data
        tcp_pose = rtde_r.getActualTCPPose()
        joint_positions = rtde_r.getActualQ()
        
        # Disable freedrive
       # disable_freedrive()
        
        # Format positions for readability
        tcp_pose_rounded = [round(value, 4) for value in tcp_pose]
        joint_positions_rounded = [round(value, 4) for value in joint_positions]
        
        # Save both TCP and joint positions
        positions[position_name] = {
            "tcp_pose": tcp_pose,  # Save exact values
            "joint_positions": joint_positions  # Save exact values
        }
        
        # Show the captured position to user
        print(f"\nPosition '{position_name}' captured successfully!")
        print(f"TCP pose: {tcp_pose_rounded}")
        print(f"Joint positions: {joint_positions_rounded}")
        
        # Save after each capture in case of unexpected termination
        with open(positions_file, 'w') as f:
            json.dump(positions, f, indent=2)
        print(f"Position saved to {positions_file}")

    print("\n==== Saved Positions ====")
    for name, data in positions.items():
        tcp_rounded = [round(value, 4) for value in data["tcp_pose"]]
        print(f"{name}: {tcp_rounded}")
        
    print(f"\nAll positions saved to {positions_file}")
    print("You can use these positions in your pick and place program.")
    
    print("\nExample usage in your program:")
    print("""
# Load positions
import json
with open("robot_positions.json", 'r') as f:
    positions = json.load(f)
    
# Use positions for pick and place
rtde_c.moveJ(positions["home"]["joint_positions"])
rtde_c.moveL(positions["pick_approach"]["tcp_pose"])
rtde_c.moveL(positions["pick"]["tcp_pose"])
# Close gripper
rtde_c.moveL(positions["place_approach"]["tcp_pose"])
rtde_c.moveL(positions["place"]["tcp_pose"])
# Open gripper
    """)
        
except KeyboardInterrupt:
    print("\nProgram interrupted.")
finally:
    # Make sure we turn off freedrive mode on exit
    try:
        rtde_c.endTeachMode()
    except:
        pass
    rtde_c.disconnect()
    rtde_r.disconnect()
