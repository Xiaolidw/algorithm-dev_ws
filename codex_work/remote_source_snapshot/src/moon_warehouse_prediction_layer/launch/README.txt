# launch 目录说明

PredictionLayer 是 costmap 插件，不提供独立启动文件 —— 它随 Nav2
costmap 一起加载（见 `config/prediction_layer_example.yaml` 的注册方式）。

本目录保留用于放置后续的冒烟测试 launch（例如启动一个往复障碍物 + RViz
查看预测成本），当前阶段无需执行任何 launch。
