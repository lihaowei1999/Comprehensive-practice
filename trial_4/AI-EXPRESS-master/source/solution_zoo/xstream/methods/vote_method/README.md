# Vote Method
## 介绍
VoteMethod 是滑动窗投票策略的封装

## 输入输出 

车型车颜色、车牌颜色
输入：

|  slot0  | boxes(框，主要是用到里边的track_id信息)  |
|  ---- | ----  |
| slot1 | disappeared_track_ids（消失的track_id,用于清理内部关于此id的资源）|
| slot2 | vote_info（需要投票的信息)|

输出：

|  slot0  | vote_info（投票后的结果）|
|  ---- | ----  |

活体
输入：

|  slot0  | boxes(框，主要是用到里边的track_id信息)  |
|  ---- | ----  |
| slot1 | disappeared_track_ids（消失的track_id,用于清理内部关于此id的资源）|
| slot2 | vote_info（需要投票的信息)|

输出：

|  slot0  | track_id |
|  ---- | ----  |
|  slot1  | vote_info（投票后的结果）|

## 补充说明
### 单实例不支持多线程访问，支持多实例。

## 配置文件描述

配置文件是config目录下的json，下面讲解配置里主要参数的意思。

type 需要投票的类型，可选 vehicle(车型车颜色)、plate_color(车牌颜色)、living(活体)

vehicle 和 plate_color 需要配置 max_slide_window_size 表示滑动窗口的大小，默认值为50.
living 需要配置 
    max_slide_window_size 默认值7
    living_voting_threshold 默认值 0.5
    fake_living_voting_threshold 默认值 0.5


## 策略简要描述

1. 判断输入的投票信息是否有效，即查看当前滑动窗口大小是否等于 max_slide_window_size,不等于设置投票信息为无效，对于车型车颜色若无效使用上一帧信息，

2. 比较当前滑动窗口大小和 max_slide_window_size, 若小于设置投票输出信息为无效，同时把需要投票的信息进行 push_back,若等于进行一次投票

3. 把投票结果设置到输出

4. 活体算法提供的策略

由于人脸优选策略以及人脸外扩卡大小策略会对每一帧进行优选，在不满足策略要求的情况下，该帧图片不会送入到活体检测模型中进行判断。在之前的滑动窗口策略中，优选不过的图片会被直接标记为非活，为了避免演示时出现因为优选策略被判为非活，进而提示用户的情况，现对滑动窗口策略进行补充和微调。

参数说明以及调整点：

活体投票阈值：当前窗口内判断为活体的帧所占比例超过活体投票阈值，则当前帧判断为活体，默认50%。

非活投票阈值：当前窗口内判断为非活的帧所占比例超过非活投票阈值，则当前帧判断为非活，默认50%。

判断流程：

    1. 首先进行活体判断，当前窗口内判断为活体的帧所占比例超过活体投票阈值，则当前帧判断为活体。

    2. 如果当前窗口内判断为活体的帧占比没有超过活体投票阈值，则进行非活判断：

        a.如果当前窗口内判断为非活的帧所占比例超过非活投票阈值，则当前帧判断为非活。

        b.如果当前窗口内判断为非活的帧所占比没有超过非活投票阈值，则不对当前帧进行判断。
