# 说明

此工程为ros下点云压缩传输解压缩，用以减少点云话题所占带宽

# 环境准备

```bash
sudo apt install -y libopencv-dev libfftw3-dev libzstd-dev libpcl-dev libboost-all-dev
```

# 点云话题编码
```bash
source devel/setup.bash
roslaunch rcpcc run_encode.launch
```
|参数名称|功能描述|备注|
|---|---|---|
|lidar_topic|接收lidar点云的topic话题| - |
|encoded_topic|发布压缩后点云数据话题| - |
|q_level|压缩率设置| 取值范围0-5，越大压缩效果越强 |

# 点云话题解码
```bash
source devel/setup.bash
roslaunch rcpcc run_decode.launch
```
|参数名称|功能描述|备注|
|---|---|---|
|lidar_restored_topic|发布解压缩的lidar点云的topic话题| - |
|encoded_topic|接收待解压缩点云数据话题| - |
|q_level|压缩率设置| 取值范围0-5，同编码设置 |

# 致谢
项目技术及源码改编来自 https://github.com/HITSZ-NRSL/RCPCC 工程