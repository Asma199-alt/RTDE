import onRobot.gripper as gripper

rg_id = 0
ip = "192.168.56.101"
rg_gripper = gripper.RG2(ip,rg_id)

rg_width = rg_gripper.get_rg_width()
print("rg_width: ",rg_width)

target_force = 40.00

rg_gripper.rg_grip(100.0, target_force)
