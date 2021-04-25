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

## 2021.4.25

重新梳理了程序的交叉编译和烧录流程。分析了现有骨架提取程序的代码，能够获得全部17个关节点的xy坐标和置信度，确定了各点和关节的对应关系。核心代码在source\solution_zoo\xstream\methods\fasterrcnnmethod\src\faster_rcnn_imp.cpp中
下周我们将研究如何将这些数据和后续的姿态识别算法衔接，并且学习Xstrem框架的使用。

此外，我们研究了Web端输出的运作方式，是将图片（jpeg）和）（x,y,置信度 ）数据发送后，用h-canvas.js中的程序实现了骨架节点和连接线的渲染，此外还有识别框等。下一步（约12周后）我们需要研究如何利用js显示统计信息。

模型文件大多在hbm中，实际上地平线公司实现了摔倒检测的功能，但目前没有完全开源，只能看到一点代码雏形。可以学习一下。