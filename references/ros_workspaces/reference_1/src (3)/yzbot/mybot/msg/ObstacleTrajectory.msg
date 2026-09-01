# 单个障碍物的未来轨迹
string id              # obstacle2 / obstacle3
string type            # obstacle_dynamic

geometry_msgs/Pose[] future_poses    # LSTM 输出的未来 (x,y,z, orientation)
float32[] future_times                # 每个预测点对应的时间（秒），与 future_poses 数量相同

