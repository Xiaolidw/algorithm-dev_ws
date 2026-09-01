# 多个物体的位姿数组（用于批量发布）
std_msgs/Header header  # 时间戳和坐标系（frame_id=world）
mybot/ObjectPose[] objects  # 物体位姿列表（引用上面的ObjectPose消息）
