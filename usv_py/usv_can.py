import can
import cantools

class CAN(can.Listener):
    # CAN 总线量
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
    battCellVoltMinID = [float("nan")] * 4 

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
            battIdx = int((msg.arbitration_id - 0x18904101) / 0x100)
            self.battCumuVolt[battIdx] = decoded_msg.get('cumulative_voltage', None)
            self.battCollVolt[battIdx] = decoded_msg.get('collecting_voltage', None)
            self.battCurrent[battIdx] = decoded_msg.get('current', None)
            self.battSOC[battIdx] = decoded_msg.get('soc', None)

        elif ("_cell_status" in message_obj.name):
            battIdx = int((msg.arbitration_id - 0x18914101) / 0x100)
            self.battCellVoltMax[battIdx] = decoded_msg.get('max_volt', None)
            self.battCellVoltMaxID[battIdx] = decoded_msg.get('max_volt_id', None)
            self.battCellVoltMin[battIdx] = decoded_msg.get('min_volt', None)
            self.battCellVoltMinID[battIdx] = decoded_msg.get('min_volt_id', None)
        else:
            pass



def main():
    usvCANBUS = CAN()
    try:
        # 主循环
        while True:
            pass
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()