import rospy
from mavros_msgs.srv import CommandLong

def play_tune():
    # 初始化ROS节点
    rospy.init_node('play_tune_node', anonymous=True)

    # 等待服务变得可用
    rospy.wait_for_service('/mavros/cmd/command')

    try:
        # 创建服务客户端
        cmd_long_service = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)

        # 定义要播放的音调字符串
        tune_string = "MFT200e8a8a"

        # 调用服务来发送MAV_CMD_DO_PLAY_TUNE命令
        response = cmd_long_service(
            False,  # broadcast
            258,    # MAV_CMD_DO_PLAY_TUNE
            0,      # confirmation
            0,      # param1 (tune format: 0 for MML, 1 for ABC)
            0.0, 0.0, 0.0, 0.0,  # unused parameters
            0.0,  # param5 (tune string)
            0.0 # param6 (tune string continuation)
        )

        # 检查响应
        if response.success:
            rospy.loginfo("Tune command sent successfully")
        else:
            rospy.logerr("Failed to send tune command: %s", response.result)

    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

if __name__ == "__main__":
    play_tune()