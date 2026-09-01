# 单个物体的位姿信息（适配导航/抓取模块需求）
string id              # 物体唯一ID（如red_cube_1、zone_a）
string type            # 物体类型（block_red/block_blue/zone）
geometry_msgs/Pose pose # 物体在world坐标系下的位姿（x/y/z + 四元数）
