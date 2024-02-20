import bpy
import time

import sys
#sys.path.append("/usr/lib/python3/dist-packages")
import serial
import glob
import numpy as np


FPS = 60
# 串口配置
port = 'COM10'  # 指定串口号，根据实际情况修改
baudrate = 115200  # 波特率，根据实际情况修改

# 创建串口对象
ser = serial.Serial(port, baudrate, timeout=1)
if ser.isOpen():
    print(f"串口 {port} 已打开，波特率 {baudrate}")
else:
    print(f"无法打开串口 {port}")

#Connect the suit first and after a ~second launch the script


# # Get the whole bge scene
# scene = bge.logic.getCurrentScene()
# # Helper vars for convenience
# source = scene.objects
# # Get the whole Armature
# main_arm = source.get('Armature')
# ob = bge.logic.getCurrentController().owner
# 获取当前场景
scene = bpy.context.scene
# 获取 'Armature' 骨骼对象
armature = bpy.data.objects["骨架"]
# 获取当前对象（这里假设是骨骼对象的所有者，可以根据实际情况修改）
ob = bpy.context.active_object

# get the bones we need
bone_upper_arm_R = armature.pose.bones.get("armUp.R")
bone_lower_arm_R = armature.pose.bones.get("armDown.R")
bone_upper_arm_L = armature.pose.bones.get("armUp.L")
bone_lower_arm_L = armature.pose.bones.get("armDown.L")
bone_trunk = armature.pose.bones.get("trunk.001")
bone_head = armature.pose.bones.get("head.001")
bone_upper_leg_R = armature.pose.bones.get("legUp.R.001")
bone_lower_leg_R = armature.pose.bones.get("legDown.R.001")
bone_upper_leg_L = armature.pose.bones.get("legUp.L.001")
bone_lower_leg_L = armature.pose.bones.get("legDown.L.001")

def multiplyQuaternion(q1, q0):
    w0, x0, y0, z0 = q0
    w1, x1, y1, z1 = q1
    return np.array([-x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
                     x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                     -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                     x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0], dtype=np.float64)

def setBoneRotation(bone, rotation):
    w, x, y, z = rotation
    bone.rotation_quaternion[0] = w
    bone.rotation_quaternion[1] = x
    bone.rotation_quaternion[2] = y
    bone.rotation_quaternion[3] = z
    
def updateAngles(angles):
    lowerarmR_out = np.array([angles[0][0],angles[0][1],angles[0][2],angles[0][3]])
    upperarmR_out = np.array([angles[1][0],angles[1][1],angles[1][2],angles[1][3]])
    lowerarmL_out = np.array([angles[2][0],angles[2][1],angles[2][2],angles[2][3]])
    upperarmL_out = np.array([angles[3][0],angles[3][1],angles[3][2],angles[3][3]])
    trunk_out = np.array((angles[4][0],angles[4][1],angles[4][2],angles[4][3]))
    upperLegR_out = np.array((angles[5][0],angles[5][1],angles[5][2],angles[5][3]))
    lowerLegR_out = np.array((angles[6][0],angles[6][1],angles[6][2],angles[6][3]))
    upperLegL_out = np.array((angles[7][0],angles[7][1],angles[7][2],angles[7][3]))
    lowerLegL_out = np.array((angles[8][0],angles[8][1],angles[8][2],angles[8][3]))
    head_out = np.array((angles[9][0],angles[9][1],angles[9][2],angles[9][3]))

#    upperarmR_inv = upperarmR_out * np.array([1, -1, -1, -1])
#    lowerarmR_rel = multiplyQuaternion(upperarmR_inv, lowerarmR_out)
    trunk_inv = trunk_out * np.array([1, -1, -1, -1])
    head_rel = multiplyQuaternion(trunk_inv, head_out)
#    upperarmR_rel = multiplyQuaternion(trunk_inv, upperarmR_out)
    upperarmR_inv = upperarmR_out * np.array([1, -1, -1, -1])
    lowerarmR_rel = multiplyQuaternion(upperarmR_inv, lowerarmR_out)
    upperarmL_inv = upperarmL_out * np.array([1, -1, -1, -1])
    lowerarmL_rel = multiplyQuaternion(upperarmL_inv, lowerarmL_out)
    upperLegR_inv = upperLegR_out * np.array([1, -1, -1, -1])
    lowerLegR_rel = multiplyQuaternion(upperLegR_inv, lowerLegR_out)
    upperLegL_inv = upperLegL_out * np.array([1, -1, -1, -1])
    lowerLegL_rel = multiplyQuaternion(upperLegL_inv, lowerLegL_out)
    
    setBoneRotation(bone_trunk, trunk_out)         
    setBoneRotation(bone_upper_arm_R, upperarmR_out)
    setBoneRotation(bone_lower_arm_R, lowerarmR_rel)
    setBoneRotation(bone_upper_arm_L, upperarmL_out)
    setBoneRotation(bone_lower_arm_L, lowerarmL_rel)
    setBoneRotation(bone_head, head_rel)
    setBoneRotation(bone_upper_leg_R, upperLegR_out)
    setBoneRotation(bone_lower_leg_R, lowerLegR_rel)
    setBoneRotation(bone_upper_leg_L, upperLegL_out)
    setBoneRotation(bone_lower_leg_L, lowerLegL_rel)
    
def resetBone():
    setBoneRotation(bone_upper_arm_R, [1, 0, 0, 0])
    setBoneRotation(bone_lower_arm_R, [1, 0, 0, 0])
    setBoneRotation(bone_upper_arm_L, [1, 0, 0, 0])
    setBoneRotation(bone_lower_arm_L, [1, 0, 0, 0])
    setBoneRotation(bone_trunk, [0.707, 0.707, 0, 0])
    setBoneRotation(bone_head, [1, 0, 0, 0])
    setBoneRotation(bone_upper_leg_R, [0.707, -0.707, 0, 0])
    setBoneRotation(bone_lower_leg_R, [1, 0, 0, 0])
    setBoneRotation(bone_upper_leg_L, [0.707, -0.707, 0, 0])
    setBoneRotation(bone_lower_leg_L, [1, 0, 0, 0])

class ModalTimerOperator(bpy.types.Operator):
    # we need these two fields for Blender
    bl_idname = "wm.modal_timer_operator"
    bl_label = "Modal Timer Operator"
    
    _timer = None
    
    def modal(self, context, event):
        if event.type == "ESC":
            print("BlenderTimer received ESC.")
            return self.cancel(context)
    
        if event.type == "TIMER":
            # this will eval true every Timer delay seconds
            # If you use interrupt mode, do not comment on this line of code
#            ser.write("a".encode('UTF-8'))
            s=ser.readline()[:-3].decode('UTF-8') # delete ";\r\n"
            print(s)
            
            if not s:
                print("Invalid joint data")
                return {"PASS_THROUGH"}
            
            angles=[x.split(',') for x in s.split(';')]
            for i in range(len(angles)):
                angles[i] = [float(x) for x in angles[i]]
            if len(angles) == 10:
                updateAngles(angles)

            # if refresh rate is too low, uncomment this line to force Blender to render viewport
#            bpy.ops.wm.redraw_timer(type="DRAW_WIN_SWAP", iterations=1)
    
        return {"PASS_THROUGH"}

    def execute(self, context):
        # update rate is 0.01 second
        self._timer = context.window_manager.event_timer_add(1./FPS, window=context.window)
        context.window_manager.modal_handler_add(self)
        return {"RUNNING_MODAL"}
        
    def cancel(self, context):
        ser.close()
        
        # reset joint position
        resetBone()
        context.window_manager.event_timer_remove(self._timer)
        print("BlenderTimer Stopped.")
        return {"CANCELLED"}


if __name__ == "__main__":
    try:
        print("Starting services.")
        bpy.utils.register_class(ModalTimerOperator)
        
        # start Blender timer
        bpy.ops.wm.modal_timer_operator()
        
        print("All started.")
    except KeyboardInterrupt:
        resetBone()
        ser.close()
        print("Received KeyboardInterrupt, stopped.")
