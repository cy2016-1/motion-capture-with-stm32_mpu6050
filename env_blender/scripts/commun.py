import bpy
import time
import serial
import threading
import json

# 串口连接类
class SerialConnection:
    def __init__(self, port, baudrate):
        self.port = port
        self.baudrate = baudrate
        self.serial = serial.Serial(port, baudrate)
        self.received_data = ""

# 打开串口
def open_serial_port():
    port = bpy.context.scene.serial_port
    baudrate = bpy.context.scene.baud_rate
    if not "serial_connection" in bpy.app.driver_namespace:
        bpy.app.driver_namespace["serial_connection"] = SerialConnection(port, baudrate)
        serial_thread = SerialReadThread(bpy.app.driver_namespace["serial_connection"])
        serial_thread.start()
        bpy.app.driver_namespace["serial_thread"] = serial_thread
        print("成功打开串口")
    else:
        print("串口已经打开")

# 串口读取线程
class SerialReadThread(threading.Thread):
    def __init__(self, serial_connection):
        threading.Thread.__init__(self)
        self.serial_connection = serial_connection
        self.should_terminate = False

    def run(self):
        while not self.should_terminate:
            if self.serial_connection.serial.in_waiting > 0:
                data = self.serial_connection.serial.readline().decode("utf-8").strip().split(',')
                # 解析接收到的数据并控制物体移动(用于刷新)

            time.sleep(0.015)

# 注册插件
def register():
    bpy.utils.register_class(OpenPortOperator)
    bpy.utils.register_class(ClosePortOperator)
    bpy.types.Scene.serial_port = bpy.props.StringProperty(name="串口", default="COM6")
    bpy.types.Scene.baud_rate = bpy.props.IntProperty(name="波特率", default=115200)

    # 添加自定义操作面板
    bpy.utils.register_class(SerialCommunicationPanel)

# 注销插件
def unregister():
    bpy.utils.unregister_class(OpenPortOperator)
    bpy.utils.unregister_class(ClosePortOperator)
    del bpy.types.Scene.serial_port
    del bpy.types.Scene.baud_rate

    # 移除自定义操作面板
    bpy.utils.unregister_class(SerialCommunicationPanel)

# 打开串口操作类
class OpenPortOperator(bpy.types.Operator):
    bl_idname = "serial.open_port_operator"
    bl_label = "打开串口"

    def execute(self, context):
        open_serial_port()
        return {'FINISHED'}

# 关闭串口操作类
class ClosePortOperator(bpy.types.Operator):
    bl_idname = "serial.close_port_operator"
    bl_label = "关闭串口"

    def execute(self, context):
        if "serial_connection" in bpy.app.driver_namespace:
            serial_thread = bpy.app.driver_namespace["serial_thread"]
            serial_thread.should_terminate = True
            del bpy.app.driver_namespace["serial_connection"]
            del bpy.app.driver_namespace["serial_thread"]
            print("成功关闭串口")
        else:
            print("无可关闭的串口")
        return {'FINISHED'}

# 串口通信面板类
class SerialCommunicationPanel(bpy.types.Panel):
    bl_label = "串口通讯"
    bl_idname = "PT_SerialCommunicationPanel"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = '串口助手'

    def draw(self, context):
        layout = self.layout

        row = layout.row()
        row.prop(context.scene, "serial_port", text="串口")

        row = layout.row()
        row.prop(context.scene, "baud_rate", text="波特率")

        row = layout.row()
        row.operator("serial.open_port_operator", text="打开串口")

        row = layout.row()  # 添加新的行布局
        row.operator("serial.close_port_operator", text="关闭串口")

# 启动插件
if __name__ == "__main__":
    register()
