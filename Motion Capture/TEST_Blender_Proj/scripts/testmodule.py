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
baudrate = 256000  # 波特率，根据实际情况修改

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
bone_0 = armature.pose.bones.get("bone")
bone_1 = armature.pose.bones.get("bone.001")
bone_2 = armature.pose.bones.get("bone.002")
bone_3 = armature.pose.bones.get("bone.003")
bone_4 = armature.pose.bones.get("bone.004")
bone_5 = armature.pose.bones.get("bone.005")
bone_6 = armature.pose.bones.get("bone.006")
bone_7 = armature.pose.bones.get("bone.007")
bone_8 = armature.pose.bones.get("bone.008")
bone_9 = armature.pose.bones.get("bone.009")


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
    bone_0_out = np.array((angles[0][0],angles[0][1],angles[0][2],angles[0][3]))
    bone_1_out = np.array((angles[1][0],angles[1][1],angles[1][2],angles[1][3]))
    bone_2_out = np.array((angles[2][0],angles[2][1],angles[2][2],angles[2][3]))
    bone_3_out = np.array((angles[3][0],angles[3][1],angles[3][2],angles[3][3]))
    bone_4_out = np.array((angles[4][0],angles[4][1],angles[4][2],angles[4][3]))
    bone_5_out = np.array((angles[5][0],angles[5][1],angles[5][2],angles[5][3]))
    bone_6_out = np.array((angles[6][0],angles[6][1],angles[6][2],angles[6][3]))
    bone_7_out = np.array((angles[7][0],angles[7][1],angles[7][2],angles[7][3]))
    bone_8_out = np.array((angles[8][0],angles[8][1],angles[8][2],angles[8][3]))
    bone_9_out = np.array((angles[9][0],angles[9][1],angles[9][2],angles[9][3]))

    setBoneRotation(bone_0, bone_0_out)
    setBoneRotation(bone_1, bone_1_out)
    setBoneRotation(bone_2, bone_2_out)
    setBoneRotation(bone_3, bone_3_out)
    setBoneRotation(bone_4, bone_4_out)
    setBoneRotation(bone_5, bone_5_out)
    setBoneRotation(bone_6, bone_6_out)
    setBoneRotation(bone_7, bone_7_out)
    setBoneRotation(bone_8, bone_8_out)
    setBoneRotation(bone_9, bone_9_out)
    
def resetBone():
    setBoneRotation(bone_0, [1, 0, 0, 0])
    setBoneRotation(bone_1, [1, 0, 0, 0])
    setBoneRotation(bone_2, [1, 0, 0, 0])
    setBoneRotation(bone_3, [1, 0, 0, 0])
    setBoneRotation(bone_4, [1, 0, 0, 0])
    setBoneRotation(bone_5, [1, 0, 0, 0])
    setBoneRotation(bone_6, [1, 0, 0, 0])
    setBoneRotation(bone_7, [1, 0, 0, 0])
    setBoneRotation(bone_8, [1, 0, 0, 0])
    setBoneRotation(bone_9, [1, 0, 0, 0])

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
