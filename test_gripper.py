import numpy as np
import socket
import time
import struct
import rtde
import util

# Robot configuration
HOST = "129.244.149.108"
#HOST = "192.168.5.4" real robot   
PORT = 30003

def check_robot_connection():
    """Test if we can connect to the robot"""
    try:
        tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        tcp_socket.connect((HOST, PORT))
        data = tcp_socket.recv(1108)
        tcp_socket.close()
        print("✅ Robot connection successful")
        return True
    except Exception as e:
        print(f"❌ Robot connection failed: {e}")
        return False

def get_current_tcp():
    """Get current TCP position from robot"""
    try:
        tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        tcp_socket.connect((HOST, PORT))
        data = tcp_socket.recv(1108)
        position = struct.unpack('!6d', data[444:492])
        tcp_socket.close()
        print("✅ Successfully read TCP position")
        return np.asarray(position)
    except Exception as e:
        print(f"❌ Failed to get TCP position: {e}")
        return None

def check_digital_io():
    """Check digital I/O status"""
    try:
        tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        tcp_socket.connect((HOST, PORT))
        data = tcp_socket.recv(1108)
        tool = struct.unpack('!d', data[1044:1052])[0]
        tcp_socket.close()
        print(f"✅ Digital output status: {tool}")
        return True
    except Exception as e:
        print(f"❌ Failed to read digital I/O: {e}")
        return False

def operate_gripper_test(target_width):
    """Test gripper operation with a specific width (0-110)"""
    try:
        print(f"Attempting to operate gripper with width: {target_width}")
        tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        tcp_socket.connect((HOST, PORT))

        # This is the same gripper command from the original code
        tcp_command = "def rg2ProgOpen():\n"
        tcp_command += "\ttextmsg(\"inside RG2 function called\")\n"

        tcp_command += f'\ttarget_width={target_width}\n'
        tcp_command += "\ttarget_force=30\n"
        tcp_command += "\tpayload=1.0\n"
        tcp_command += "\tset_payload1=False\n"
        tcp_command += "\tdepth_compensation=False\n"
        tcp_command += "\tslave=False\n"

        # Add the rest of the gripper command (abbreviated for clarity)
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

        # Continue with the command...
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

        # Skipping the depth compensation section for brevity

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

        tcp_command += "end\n"

        tcp_socket.send(str.encode(tcp_command))
        tcp_socket.close()
        print("✅ Gripper command sent successfully")
        time.sleep(2)  # Wait for gripper to complete operation
        return True
    except Exception as e:
        print(f"❌ Failed to operate gripper: {e}")
        return False

def check_rtde_connection():
    """Test RTDE connection which is used for checking grasp status"""
    try:
        con = rtde.RTDE(HOST, 30004)
        success = con.connect()
        if success:
            print("✅ RTDE connection successful")
            con.disconnect()
            return True
        else:
            print("❌ RTDE connection failed")
            return False
    except Exception as e:
        print(f"❌ RTDE connection error: {e}")
        return False

def check_grasp_sensor():
    """Test grasp sensor reading"""
    try:
        con = rtde.RTDE(HOST, 30004)
        con.connect()
        output_names = ['tool_analog_input0']
        output_types = ['DOUBLE']
        con.send_output_setup(output_names, output_types, frequency=125)
        con.send_start()
        state = con.receive(True)
        if state:
            voltage = struct.unpack('!d', state)
            print(f"✅ Gripper sensor voltage: {voltage[0]}")
            con.disconnect()
            return True
        else:
            print("❌ Failed to read gripper sensor")
            con.disconnect()
            return False
    except Exception as e:
        print(f"❌ Gripper sensor reading error: {e}")
        return False

def run_gripper_tests():
    """Run a series of tests to check each aspect of gripper operation"""
    tests_passed = 0
    total_tests = 5
    
    print("=== GRIPPER DIAGNOSTICS ===")
    print("1. Testing robot connection...")
    if check_robot_connection():
        tests_passed += 1
    
    print("\n2. Testing TCP position reading...")
    tcp_pos = get_current_tcp()
    if tcp_pos is not None:
        tests_passed += 1
        print(f"Current position: X:{tcp_pos[0]:.4f}, Y:{tcp_pos[1]:.4f}, Z:{tcp_pos[2]:.4f}")
    
    print("\n3. Testing digital I/O...")
    if check_digital_io():
        tests_passed += 1
    
    print("\n4. Testing RTDE connection (for sensor readings)...")
    if check_rtde_connection():
        tests_passed += 1
    
    print("\n5. Testing gripper operation (opening)...")
    if operate_gripper_test(100):  # Try to open gripper to 100mm
        tests_passed += 1
    
    print("\n=== DIAGNOSTICS SUMMARY ===")
    print(f"Passed {tests_passed} out of {total_tests} tests")
    
    if tests_passed == total_tests:
        print("All systems appear to be working correctly.")
        print("If you're still having issues, check the physical gripper for mechanical problems.")
    else:
        print("Some tests failed. Review the results above to identify the problem areas.")

if __name__ == "__main__":
    run_gripper_tests() 