# 调试笔记

## 2021.3.21

需要在视频心率之前，识别出人脸区域。

尝试使用opencv cpp进行视频人脸检测

配置带扩展包的opencv见： https://blog.csdn.net/weixin_46135347/article/details/114190250

由于dnn对protobuf的版本由要求，还需要手动把protobuf弄进去，所以选择直接取消dnn模块的变异（方法：在cmake config里搜索dnn，全部取消勾选）

视频人脸区域识别参考代码：https://blog.csdn.net/u012679707/article/details/80393486 ，然后把前置摄像头改成读取视频。

需要额外下载nose和mouth检测的xml文件，放git里了： asset\haar_cascade\data\haarcascades

目前能比较准确地识别出单个人脸和两只眼睛。

## 2021.4.3

照着lhw的教程和网上的教程，把交叉编译跑通了。等第九周周末再上板子。

## 2021.4.13

根据和腾老师之前的交流，完善了开题报告的项目背景部分。

