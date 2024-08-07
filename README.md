# 单目 RGB 视觉 SLAM 系统
## 系统运行效果
![运行效果](./assets/demo.gif "运行效果")

## 所需环境
- OpenCV
- Eigen
- G2O
- Pangolin
- yaml-cpp
- CSparse
- FBoW

## 数据格式
将有序的图像序列放入自定义的数据文件夹中，按照格式创建配置文件。

配置文件包含**相机内参系数**、**特征提取参数**、**建图参数**、**可视化参数**相关设置。

主要需要修改**相机内参系数**，可通过OpenCV相机标定相关功能获得。

## 如何运行
进入项目文件夹，打开终端执行如下命令编译项目。

` mkdir build `

` cd build `

` cmake .. `

` make -j4 `

执行如下命令运行项目

` ./visual-slam -v ../vocb/orb_vocab.fbow -i [图像序列所在目录] -c [配置文件路径] `
