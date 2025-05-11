import cv2
import numpy as np
from PIL import Image

def extract_frames_with_timestamps(video_path, interval_sec=0.5):
    """
    从视频中按指定时间间隔提取帧及其时间戳
    :param video_path: 视频文件路径
    :param interval_sec: 提取帧的时间间隔(秒)
    :return: (帧列表, 时间戳列表)
    """
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        raise ValueError("无法打开视频文件")
    
    fps = cap.get(cv2.CAP_PROP_FPS)
    frame_interval = int(fps * interval_sec)
    frames = []
    timestamps = []
    
    frame_count = 0
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        if frame_count % frame_interval == 0:
            # 将BGR转换为RGB
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            frames.append(frame_rgb)
            # 计算当前帧的时间戳(秒)
            timestamp = frame_count / fps
            timestamps.append(timestamp)
        
        frame_count += 1
    
    cap.release()
    return frames, timestamps

def blend_frames_with_variable_alpha(frames, timestamps, alpha_before=0.3, alpha_after=0.1, split_time=10):
    """
    将多帧图像以不同透明度叠加
    :param frames: 帧列表(numpy数组)
    :param timestamps: 对应的时间戳列表(秒)
    :param alpha_before: 分割时间点前的透明度
    :param alpha_after: 分割时间点后的透明度
    :param split_time: 分割时间点(秒)
    :return: 叠加后的图像(Pillow Image对象)
    """
    if not frames:
        raise ValueError("没有可用的帧")
    if len(frames) != len(timestamps):
        raise ValueError("帧数和时间戳数不一致")
    
    # 初始化叠加图像
    blended = np.zeros_like(frames[0], dtype=np.float32)
    
    for frame, timestamp in zip(frames, timestamps):
        frame_float = frame.astype(np.float32)
        # 根据时间戳决定使用哪个透明度
        alpha = alpha_before if timestamp < split_time else alpha_after
        blended += frame_float * alpha
    
    # 转换回uint8并创建Pillow图像
    blended = np.clip(blended, 0, 255).astype(np.uint8)
    return Image.fromarray(blended)

def process_video_with_variable_alpha(video_path, output_path, interval_sec=0.5, alpha_before=0.3, alpha_after=0.1, split_time=10):
    """
    处理视频并生成运动轨迹图像(使用不同透明度)
    :param video_path: 输入视频路径
    :param output_path: 输出图像路径
    :param interval_sec: 帧提取间隔(秒)
    :param alpha_before: 分割时间点前的透明度
    :param alpha_after: 分割时间点后的透明度
    :param split_time: 分割时间点(秒)
    """
    # 提取帧及其时间戳
    frames, timestamps = extract_frames_with_timestamps(video_path, interval_sec)
    
    # 叠加帧(使用不同透明度)
    result_image = blend_frames_with_variable_alpha(frames, timestamps, alpha_before, alpha_after, split_time)
    
    # 保存结果
    result_image.save(output_path)
    print(f"轨迹图像已保存到: {output_path}")

# 使用示例
if __name__ == "__main__":
    input_video = "robot.mp4"  # 替换为你的视频文件路径
    output_image = "robot_trajectory_variable_alpha2.png"
    
    process_video_with_variable_alpha(
        video_path=input_video,
        output_path=output_image,
        interval_sec=1.2,  # 每0.5秒提取一帧
        alpha_before=0.12,  # 前10秒的透明度
        alpha_after=0.12,   # 10秒后的透明度
        split_time=10      # 10秒为分界点
    )