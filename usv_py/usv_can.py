import can
import cantools

class CAN(can.Listener):
    # CAN 总线设置
    canInterface = 'vcan0'
    canBusType = 'socketcan'
    dbcPath = '/home/kk/battery_can.dbc'

    # CAN 可监听到的 USV 状态量
    battCumuVolt = [float("nan")] * 4
    battCollVolt = [float("nan")] * 4
    battCurrent = [float("nan")] * 4
    battSOC = [float("nan")] * 4
    battCellVoltMax = [float("nan")] * 4 
    battCellVoltMaxID = [float("nan")] * 4 
    battCellVoltMin = [float("nan")] * 4 
    battCellVoltMinID = [float("nan")] *4

    motorRPM = [-9999] * 2
    motorAngle = [float("nan")] * 2

    def __init__(self):
        # 加载 DBC 文件
        self.db = cantools.database.load_file(self.dbcPath)

        # 提取 DBC 文件所有的 CAN ID
        canIDs = [msg.frame_id for msg in self.db.messages]

        # 创建过滤器
        self.canFilters = [{'can_id': canID, 'can_mask': 0x1FFFFFFF if canID > 0x7FF else 0x7FF} for canID in canIDs]
        
        # 创建一个 CAN 总线实例
        self.bus = can.interface.Bus(channel=self.canInterface, bustype=self.canBusType, can_filters=self.canFilters)

        # 创建一个 CAN 的提醒器，将其与总线和监听器关联起来
        self.notifier = can.Notifier(self.bus, [self])

    def on_message_received(self, msg):
        # 获取消息对象
        message_obj = self.db.get_message_by_frame_id(msg.arbitration_id)
        
        # 解码整个消息
        decoded_msg = message_obj.decode(msg.data)

        # 分类信息来源
        if ("_overall_status" in message_obj.name):
            # 电池设备 CAN ID （eg: 0x18904101）与掩码 0x00000F00 取并，所得结果在二进制形式下向右移动 8 位，减去 1，得到是第几个电池
            battIdx = ((msg.arbitration_id & 0x00000F00)>>8) - 1
            self.battCumuVolt[battIdx] = decoded_msg.get('cumulative_voltage', None) * 0.1
            self.battCollVolt[battIdx] = decoded_msg.get('collecting_voltage', None) * 0.1
            self.battCurrent[battIdx] = decoded_msg.get('current', None) * 0.1
            self.battSOC[battIdx] = decoded_msg.get('soc', None) * 0.1

        elif ("_cell_status" in message_obj.name):
            battIdx = ((msg.arbitration_id & 0x00000F00)>>8) - 1
            self.battCellVoltMax[battIdx] = decoded_msg.get('max_volt', None) * 1e-3
            self.battCellVoltMaxID[battIdx] = decoded_msg.get('max_volt_id', None)
            self.battCellVoltMin[battIdx] = decoded_msg.get('min_volt', None) * 1e-3
            self.battCellVoltMinID[battIdx] = decoded_msg.get('min_volt_id', None)
        else:
            pass

if __name__ == "__main__":
    usvCANBUS = CAN()
    try:
        # 主循环
        while True:
            pass
    except KeyboardInterrupt:
        pass