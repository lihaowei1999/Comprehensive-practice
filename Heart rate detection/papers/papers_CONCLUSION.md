## HR and HRV from single-channel video and ica integration of mutiple signals

测量目标：HR, HRV(rmssd)

方法：$560\pm 20nm$绿光滤波,后期直接转灰度，ICA预处理（独立成分分析）

![image-20210315145740759](C:\Users\lihaowei\Documents\GitHub\Comprehensive-practice\Heart rate detection\papers\papers_CONCLUSION.assets\image-20210315145740759.png)

白色环形灯打光，中央摄影，同时测量ECG

![image-20210315150104368](C:\Users\lihaowei\Documents\GitHub\Comprehensive-practice\Heart rate detection\papers\papers_CONCLUSION.assets\image-20210315150104368.png)

其中，i为相对光移动产生的影响，m为皮肤表面运动产生的影响，p是散射的改变

![image-20210315150210989](C:\Users\lihaowei\Documents\GitHub\Comprehensive-practice\Heart rate detection\papers\papers_CONCLUSION.assets\image-20210315150210989.png)

这里用ICA处理

数据来源：12女18男。

![image-20210315153740934](C:\Users\lihaowei\Documents\GitHub\Comprehensive-practice\Heart rate detection\papers\papers_CONCLUSION.assets\image-20210315153740934.png)

ROI的def，cascading classifier，l=20PIXEL

133Hz imaging，300Hz ECG

三通道取样，白化，ICA，得到频谱在0.75-2Hz内Peak值最大的为血流像



## Algorithms for Monitoring Heart Rate andRespiratory Rate From the Video of a User’s Face

![image-20210315170240406](C:\Users\lihaowei\Documents\GitHub\Comprehensive-practice\Heart rate detection\papers\papers_CONCLUSION.assets\image-20210315170240406.png)

血红蛋白对光的反应，集中在绿光

已实现

RGB转HSV空间，获取接近皮肤颜色的H范围，观察H的数值波动



