import numpy as np
import socket
import time
import struct
import util
import rtde
import json

HOST = "129.244.149.108" # ursimr robot ip
PORT = 30003


def get_current_tcp():
    tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    tcp_socket.connect((HOST, PORT))
    data = tcp_socket.recv(1108)
    position = struct.unpack('!6d', data[444:492])
    tcp_socket.close()
    return np.asarray(position)


def get_current_pos():  # x, y, theta
    tcp = get_current_tcp()
    rpy = util.rv2rpy(tcp[3], tcp[4], tcp[5])
    return np.asarray([tcp[0], tcp[1], rpy[-1]])


def get_current_pos_same_with_simulation():
    tcp = get_current_tcp()
    rpy = util.rv2rpy(tcp[3], tcp[4], tcp[5])
    return np.asarray([tcp[1], tcp[0], rpy[-1]])


    #move to tcp takes target_tcp as input
def move_to_tcp(target_tcp):
    tool_acc = 0.05  # Safe: 0.5
    tool_vel = 0.05 # Safe: 0.2
    tool_pos_tolerance = [0.001, 0.001, 0.001, 0.05, 0.05, 0.05]
    tcp_command = "movel(p[%f,%f,%f,%f,%f,%f],a=%f,v=%f,t=0,r=0)\n" % (
        target_tcp[0], target_tcp[1], target_tcp[2], target_tcp[3], target_tcp[4],
        target_tcp[5],
        tool_acc, tool_vel)
    tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    tcp_socket.connect((HOST, PORT))
    tcp_socket.send(str.encode(tcp_command))  
    tcp_socket.close()
   
    actual_pos = get_current_tcp()
    target_rpy = util.rv2rpy(target_tcp[3], target_tcp[4], target_tcp[5])
    rpy = util.rv2rpy(actual_pos[3], actual_pos[4], actual_pos[5])
    while not (all([np.abs(actual_pos[j] - target_tcp[j]) < tool_pos_tolerance[j] for j in range(3)])
               and all([np.abs(rpy[j] - target_rpy[j]) < tool_pos_tolerance[j+3] for j in range(3)])):
        actual_pos = get_current_tcp()
        rpy = util.rv2rpy(actual_pos[3], actual_pos[4], actual_pos[5])
        time.sleep(0.01)


def increase_move(delta_x, delta_y, delta_z, delta_theta):
    tcp = get_current_tcp()
    rpy = util.rv2rpy(tcp[3], tcp[4], tcp[5])
    rpy[2] = rpy[2] + delta_theta
    target_rv = util.rpy2rv(rpy)
    target_tcp = np.asarray([tcp[0] + delta_x, tcp[1] + delta_y, tcp[2] + delta_z,
                             target_rv[0], target_rv[1], target_rv[2]])
    move_to_tcp(target_tcp)


def get_digital_output():
    tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    tcp_socket.connect((HOST, PORT))
    data = tcp_socket.recv(1108)
    tool = struct.unpack('!d', data[1044:1052])[0]
    tcp_socket.close()
    return tool

#open and close gripper takes target_width as input
def operate_gripper(target_width):
    tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    tcp_socket.connect((HOST, PORT))

    tcp_command = "def rg2ProgOpen():\n"
    tcp_command += "\ttextmsg(\"inside RG2 function called\")\n"

    tcp_command += '\ttarget_width={}\n'.format(target_width)
    tcp_command += "\ttarget_force=40\n"
    tcp_command += "\tpayload=1.0\n"
    tcp_command += "\tset_payload1=False\n"
    tcp_command += "\tdepth_compensation=False\n"
    tcp_command += "\tslave=False\n"

    tcp_command += "\ttimeout = 0\n"
    tcp_command += "\twhile get_digital_in(9) == False:\n"
    tcp_command += "\t\ttextmsg(\"inside while\")\n"
    tcp_command += "\t\tif timeout > 400:\n"
    tcp_command += "\t\t\tbreak\n"
    tcp_command += "\t\tend\n"
    tcp_command += "\t\ttimeout = timeout+1\n"
    tcp_command += "\t\tsync()\n"
    tcp_command += "\tend\n"
    tcp_command += "\ttextmsg(\"outside while\")\n"

    tcp_command += "\tdef bit(input):\n"
    tcp_command += "\t\tmsb=65536\n"
    tcp_command += "\t\tlocal i=0\n"
    tcp_command += "\t\tlocal output=0\n"
    tcp_command += "\t\twhile i<17:\n"
    tcp_command += "\t\t\tset_digital_out(8,True)\n"
    tcp_command += "\t\t\tif input>=msb:\n"
    tcp_command += "\t\t\t\tinput=input-msb\n"
    tcp_command += "\t\t\t\tset_digital_out(9,False)\n"
    tcp_command += "\t\t\telse:\n"
    tcp_command += "\t\t\t\tset_digital_out(9,True)\n"
    tcp_command += "\t\t\tend\n"
    tcp_command += "\t\t\tif get_digital_in(8):\n"
    tcp_command += "\t\t\t\tout=1\n"
    tcp_command += "\t\t\tend\n"
    tcp_command += "\t\t\tsync()\n"
    tcp_command += "\t\t\tset_digital_out(8,False)\n"
    tcp_command += "\t\t\tsync()\n"
    tcp_command += "\t\t\tinput=input*2\n"
    tcp_command += "\t\t\toutput=output*2\n"
    tcp_command += "\t\t\ti=i+1\n"
    tcp_command += "\t\tend\n"
    tcp_command += "\t\treturn output\n"
    tcp_command += "\tend\n"
    tcp_command += "\ttextmsg(\"outside bit definition\")\n"

    tcp_command += "\ttarget_width=target_width+0.0\n"
    tcp_command += "\tif target_force>40:\n"
    tcp_command += "\t\ttarget_force=40\n"
    tcp_command += "\tend\n"

    tcp_command += "\tif target_force<4:\n"
    tcp_command += "\t\ttarget_force=4\n"
    tcp_command += "\tend\n"
    tcp_command += "\tif target_width>110:\n"
    tcp_command += "\t\ttarget_width=110\n"
    tcp_command += "\tend\n"
    tcp_command += "\tif target_width<0:\n"
    tcp_command += "\t\ttarget_width=0\n"
    tcp_command += "\tend\n"
    tcp_command += "\trg_data=floor(target_width)*4\n"
    tcp_command += "\trg_data=rg_data+floor(target_force/2)*4*111\n"
    tcp_command += "\tif slave:\n"
    tcp_command += "\t\trg_data=rg_data+16384\n"
    tcp_command += "\tend\n"

    tcp_command += "\ttextmsg(\"about to call bit\")\n"
    tcp_command += "\tbit(rg_data)\n"
    tcp_command += "\ttextmsg(\"called bit\")\n"

    tcp_command += "\tif depth_compensation:\n"
    tcp_command += "\t\tfinger_length = 55.0/1000\n"
    tcp_command += "\t\tfinger_heigth_disp = 5.0/1000\n"
    tcp_command += "\t\tcenter_displacement = 7.5/1000\n"

    tcp_command += "\t\tstart_pose = get_forward_kin()\n"
    tcp_command += "\t\tset_analog_inputrange(2, 1)\n"
    tcp_command += "\t\tzscale = (get_analog_in(2)-0.026)/2.976\n"
    tcp_command += "\t\tzangle = zscale*1.57079633-0.087266462\n"
    tcp_command += "\t\tzwidth = 5+110*sin(zangle)\n"

    tcp_command += "\t\tstart_depth = cos(zangle)*finger_length\n"

    tcp_command += "\t\tsync()\n"
    tcp_command += "\t\tsync()\n"
    tcp_command += "\t\ttimeout = 0\n"

    tcp_command += "\t\twhile get_digital_in(9) == True:\n"
    tcp_command += "\t\t\ttimeout=timeout+1\n"
    tcp_command += "\t\t\tsync()\n"
    tcp_command += "\t\t\tif timeout > 20:\n"
    tcp_command += "\t\t\t\tbreak\n"
    tcp_command += "\t\t\tend\n"
    tcp_command += "\t\tend\n"
    tcp_command += "\t\ttimeout = 0\n"
    tcp_command += "\t\twhile get_digital_in(9) == False:\n"
    tcp_command += "\t\t\tzscale = (get_analog_in(2)-0.026)/2.976\n"
    tcp_command += "\t\t\tzangle = zscale*1.57079633-0.087266462\n"
    tcp_command += "\t\t\tzwidth = 5+110*sin(zangle)\n"
    tcp_command += "\t\t\tmeasure_depth = cos(zangle)*finger_length\n"
    tcp_command += "\t\t\tcompensation_depth = (measure_depth - start_depth)\n"
    tcp_command += "\t\t\ttarget_pose = pose_trans(start_pose,p[0,0,-compensation_depth,0,0,0])\n"
    tcp_command += "\t\t\tif timeout > 400:\n"
    tcp_command += "\t\t\t\tbreak\n"
    tcp_command += "\t\t\tend\n"
    tcp_command += "\t\t\ttimeout=timeout+1\n"
    tcp_command += "\t\t\tservoj(get_inverse_kin(target_pose),0,0,0.008,0.033,1700)\n"
    tcp_command += "\t\tend\n"
    tcp_command += "\t\tnspeed = norm(get_actual_tcp_speed())\n"
    tcp_command += "\t\twhile nspeed > 0.001:\n"
    tcp_command += "\t\t\tservoj(get_inverse_kin(target_pose),0,0,0.008,0.033,1700)\n"
    tcp_command += "\t\t\tnspeed = norm(get_actual_tcp_speed())\n"
    tcp_command += "\t\tend\n"
    tcp_command += "\tend\n"
    tcp_command += "\tif depth_compensation==False:\n"
    tcp_command += "\t\ttimeout = 0\n"
    tcp_command += "\t\twhile get_digital_in(9) == True:\n"
    tcp_command += "\t\t\ttimeout = timeout+1\n"
    tcp_command += "\t\t\tsync()\n"
    tcp_command += "\t\t\tif timeout > 20:\n"
    tcp_command += "\t\t\t\tbreak\n"
    tcp_command += "\t\t\tend\n"
    tcp_command += "\t\tend\n"
    tcp_command += "\t\ttimeout = 0\n"
    tcp_command += "\t\twhile get_digital_in(9) == False:\n"
    tcp_command += "\t\t\ttimeout = timeout+1\n"
    tcp_command += "\t\t\tsync()\n"
    tcp_command += "\t\t\tif timeout > 400:\n"
    tcp_command += "\t\t\t\tbreak\n"
    tcp_command += "\t\t\tend\n"
    tcp_command += "\t\tend\n"
    tcp_command += "\tend\n"
    tcp_command += "\tif set_payload1:\n"
    tcp_command += "\t\tif slave:\n"
    tcp_command += "\t\t\tif get_analog_in(3) < 2:\n"
    tcp_command += "\t\t\t\tzslam=0\n"
    tcp_command += "\t\t\telse:\n"
    tcp_command += "\t\t\t\tzslam=payload\n"
    tcp_command += "\t\t\tend\n"
    tcp_command += "\t\telse:\n"
    tcp_command += "\t\t\tif get_digital_in(8) == False:\n"
    tcp_command += "\t\t\t\tzmasm=0\n"
    tcp_command += "\t\t\telse:\n"
    tcp_command += "\t\t\t\tzmasm=payload\n"
    tcp_command += "\t\t\tend\n"
    tcp_command += "\t\tend\n"
    tcp_command += "\t\tzsysm=0.0\n"
    tcp_command += "\t\tzload=zmasm+zslam+zsysm\n"
    tcp_command += "\t\tset_payload(zload)\n"
    tcp_command += "\tend\n"

    tcp_command += "end\n"

    tcp_socket.send(str.encode(tcp_command))  # 利用字符串的encode方法编码成bytes，默认为utf-8类型
    tcp_socket.close()
    time.sleep(1)
    # gripper_fully_closed = check_grasp()


def check_grasp():
    con = rtde.RTDE(HOST, 30004)
    con.connect()
    output_names = ['tool_analog_input0']
    output_types = ['DOUBLE']
    con.send_output_setup(output_names, output_types, frequency=125)
    con.send_start()
    state = con.receive(True)
    voltage = struct.unpack('!d', state)
    return voltage[0] > 0.3


def move_down():
    tcp = get_current_tcp()
    tcp[2] = 0.065
    move_to_tcp(tcp)


def move_up():
    tcp = get_current_tcp()
    tcp[2] = 0.13
    move_to_tcp(tcp)


def grasp():
    operate_gripper(100)
    move_down()
    operate_gripper(0)
    move_up()
    return check_grasp()


def go_home():
    # operate_gripper(100)
    move_to_tcp([0, -0.47, 0.13, 3.14165, 0., 0.])


def test_robot_movement():
    # First, print current position
    initial_pos = get_current_tcp()
    print("Initial TCP position:")
    print(f"X: {initial_pos[0]:.4f}, Y: {initial_pos[1]:.4f}, Z: {initial_pos[2]:.4f}")
    print(f"RX: {initial_pos[3]:.4f}, RY: {initial_pos[4]:.4f}, RZ: {initial_pos[5]:.4f}")
    
    # Move to home position for a safe starting point
    print("\nMoving to home position...")
    go_home()
    
    # Get position after moving home to verify
    home_pos = get_current_tcp()
    print("Home position reached:")
    print(f"X: {home_pos[0]:.4f}, Y: {home_pos[1]:.4f}, Z: {home_pos[2]:.4f}")
    print(f"RX: {home_pos[3]:.4f}, RY: {home_pos[4]:.4f}, RZ: {home_pos[5]:.4f}")
    
    # Define a small test movement (10cm to the right from home)
    test_pos = home_pos.copy()
    test_pos[0] += 0.1  # Move 10cm in X direction
    
    # Move to test position
    print("\nMoving to test position...")
    move_to_tcp(test_pos)
    
    # Verify we reached the test position
    final_pos = get_current_tcp()
    print("Test position reached:")
    print(f"X: {final_pos[0]:.4f}, Y: {final_pos[1]:.4f}, Z: {final_pos[2]:.4f}")
    print(f"RX: {final_pos[3]:.4f}, RY: {final_pos[4]:.4f}, RZ: {final_pos[5]:.4f}")
    
    # Calculate error
    error = np.abs(final_pos - test_pos)
    print("\nPosition error:")
    print(f"X: {error[0]:.4f}, Y: {error[1]:.4f}, Z: {error[2]:.4f}")
    print(f"RX: {error[3]:.4f}, RY: {error[4]:.4f}, RZ: {error[5]:.4f}")
    
    # Return to home
    print("\nReturning to home position...")
    go_home()


#move to joint takes target_tcp as input 
def move_to_joint(target_tcp):
    """
    Move the robot to a target TCP position using joint movement (movej).
    This follows a joint-interpolated path instead of a linear path.
    """
    joint_acc = 0.1  # Joint acceleration (rad/s^2)
    joint_vel = 0.1  # Joint velocity (rad/s)
    tool_pos_tolerance = [0.001, 0.001, 0.001, 0.05, 0.05, 0.05]
    
    # Use movej with p[] for Cartesian positions
    tcp_command = "movej(p[%f,%f,%f,%f,%f,%f],a=%f,v=%f,t=0,r=0)\n" % (
        target_tcp[0], target_tcp[1], target_tcp[2], target_tcp[3], target_tcp[4],
        target_tcp[5],
        joint_acc, joint_vel)
    
    tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    tcp_socket.connect((HOST, PORT))
    tcp_socket.send(str.encode(tcp_command))
    tcp_socket.close()
    
    # Wait for movement to complete and verify position
    actual_pos = get_current_tcp()
    target_rpy = util.rv2rpy(target_tcp[3], target_tcp[4], target_tcp[5])
    rpy = util.rv2rpy(actual_pos[3], actual_pos[4], actual_pos[5])
    while not (all([np.abs(actual_pos[j] - target_tcp[j]) < tool_pos_tolerance[j] for j in range(3)])
               and all([np.abs(rpy[j] - target_rpy[j]) < tool_pos_tolerance[j+3] for j in range(3)])):
        actual_pos = get_current_tcp()
        rpy = util.rv2rpy(actual_pos[3], actual_pos[4], actual_pos[5])
        time.sleep(0.01)

#pick and place with saved positions from json file
def pick_and_place_with_saved_positions():
    # Load positions from JSON file
    with open('robot_positions.json', 'r') as f:
        positions = json.load(f)
    
    # Extract the TCP poses for pick, place, and home
    home_position = np.array(positions["home_main_final"]["tcp_pose"])
     
    place_1 = np.array(positions["1"]["tcp_pose"])
    place_2 = np.array(positions["2"]["tcp_pose"])
    place_3 = np.array(positions["3"]["tcp_pose"])
    place_4 = np.array(positions["4"]["tcp_pose"])
    place_5 = np.array(positions["5"]["tcp_pose"])

    #move to place 1
    print("Approaching place 1...")
    move_to_joint(place_1)
    
    #pick place 2
    print("Picking place 2...")
    # operate_gripper(100)  # Open gripper fully
    move_to_joint(place_2)  # Move to exact pick position
    time.sleep(0.5)  # Brief pause to ensure position is reached
    # operate_gripper(0)  # Close gripper
    
    
    #pick place 3
    print("Picking place 3...")
    move_to_joint(place_3)
    time.sleep(0.5)  # Brief pause to ensure position is reached

    #pick place 4
    print("Picking object...")
    # operate_gripper(100)  # Open gripper fully
    move_to_joint(place_4)  # Move to exact pick position
    time.sleep(0.5)  # Brief pause to ensure position is reached
    # operate_gripper(0)  # Close gripper

    #pick place 5
    print("Picking object...")
    # operate_gripper(100)  # Open gripper fully
    move_to_joint(place_5)  # Move to exact pick position
    time.sleep(0.5)  # Brief pause to ensure position is reached
    # operate_gripper(0)  # Close gripper
    
    
    #move to home
    time.sleep(0.5) 
    print("Returning to home position...")
    move_to_joint(home_position)
    
    # print("Pick and place operation completed!")
    # return True


def test_gripper():
    """
    Test the gripper by opening and closing it, with proper error handling
    """
    print("=== GRIPPER TEST ===")
    
    try:
        # Test 1: Open gripper fully
        print("Test 1: Opening gripper fully (100mm)...")
        operate_gripper(100)
        print("Gripper opened successfully")
        time.sleep(1)  # Wait to observe
        
        # Test 2: Close gripper completely
        print("Test 2: Closing gripper completely (0mm)...")
        operate_gripper(0)
        print("Gripper closed successfully")
        time.sleep(1)  # Wait to observe
        
        # Test 3: Open to 50% (mid-position)
        print("Test 3: Opening gripper to 50% (55mm)...")
        operate_gripper(55)
        print("Gripper set to 55mm successfully")
        time.sleep(1)  # Wait to observe
        
        # Test 4: Check grasp functionality
        print("Test 4: Testing check_grasp() function...")
        is_grasping = check_grasp()
        print(f"Grasp detected: {is_grasping}")
        
        # Test 5: Complete grasp cycle
        print("Test 5: Testing complete grasp() cycle...")
        print("Opening gripper, moving down, closing, moving up...")
        success = grasp()
        if success:
            print("Grasp cycle completed successfully and object detected!")
        else:
            print("Grasp cycle completed but no object detected.")
        
        print("\nGripper tests completed successfully!")
        return True
        
    except socket.error as e:
        print(f"Network/Connection Error: {e}")
        print("Check if the robot is powered on and network connection is stable.")
        return False
    except struct.error as e:
        print(f"Data Unpacking Error: {e}")
        print("Received unexpected data format from the robot.")
        return False
    except Exception as e:
        print(f"Unexpected Error: {e}")
        print("Please check robot status and try again.")
        return False


if __name__ == '__main__':
    try:
        #moving robot to 5 different places from robot_positions.json
        pick_and_place_with_saved_positions()
       
    except Exception as e:
        print(f"Critical error in main program: {e}")


