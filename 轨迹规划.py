import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation as R
from typing import List, Union, Sequence
from matplotlib.colors import LinearSegmentedColormap
# import matplotlib
# matplotlib.use('Agg')
# import matplotlib.pyplot as plt

def generate_multi_point_6d_trajectory_enhanced(
    waypoints: Sequence[Union[List[float], np.ndarray]],
    nums: Union[int, Sequence[int]] = 100,
    angle_type: str = 'zyx',
    show_all_points: bool = True,
    show_orientation: bool = True,
    orientation_skip: int = 5,
    color_map: str = 'viridis'
) -> np.ndarray:
    """
    改进版多转折点六维笛卡尔坐标轨迹生成（修复可视化问题）
    
    参数:
        waypoints: 路径点序列 [[x,y,z,a,b,c], ...] (a,b,c为欧拉角，单位度)
        nums: 每段的中间点数(整数或与waypoints长度-1相同的序列)
        angle_type: 欧拉角顺序 ('zyx', 'xyz'等)
        show_all_points: 是否显示所有中间点
        show_orientation: 是否显示姿态坐标系
        orientation_skip: 姿态坐标系的显示间隔(确保每N个点显示一个)
        color_map: 使用的颜色图谱名称
    
    返回:
        numpy数组: (N, 6)的轨迹点数组(包含所有路径点)
    """
    # 输入验证
    assert len(waypoints) >= 2, "至少需要两个路径点"
    waypoints = [np.array(wp, dtype=float) for wp in waypoints]
    assert all(len(wp) == 6 for wp in waypoints), "每个路径点必须是6维坐标"
    
    # 处理nums参数
    if isinstance(nums, int):
        nums = [nums] * (len(waypoints) - 1)
    assert len(nums) == len(waypoints) - 1, "nums长度必须等于路径点数减1"
    
    # 初始化轨迹列表
    full_trajectory = []
    segment_info = []  # 存储每段信息用于可视化
    
    # 生成每段轨迹
    for i in range(len(waypoints) - 1):
        start = waypoints[i]
        end = waypoints[i + 1]
        num = nums[i]
        
        # 将角度转换为弧度用于计算
        start_rad = start.copy()
        end_rad = end.copy()
        start_rad[3:] = np.deg2rad(start[3:])
        end_rad[3:] = np.deg2rad(end[3:])
        
        # 生成插值参数t (0到1之间)
        t_values = np.linspace(0, 1, num + 2)  # 包含起点和终点
        
        # 初始化轨迹数组（弧度）
        trajectory_rad = np.zeros((num + 2, 6))
        
        # ===== 位置部分 =====
        trajectory_rad[:, :3] = start_rad[:3] + t_values.reshape(-1, 1) * (end_rad[:3] - start_rad[:3])
        
        # ===== 姿态部分 =====
        # 使用四元数插值确保平滑旋转
        rot_start = R.from_euler(angle_type, start_rad[3:])
        rot_end = R.from_euler(angle_type, end_rad[3:])
        
        # 生成四元数插值
        quats = np.array([(1-t)*rot_start.as_quat() + t*rot_end.as_quat() for t in t_values])
        quats /= np.linalg.norm(quats, axis=1)[:, np.newaxis]  # 归一化
        
        # 转换回欧拉角并确保连续性
        rotations = R.from_quat(quats)
        euler_angles_rad = rotations.as_euler(angle_type)
        for k in range(1, len(euler_angles_rad)):
            for j in range(3):
                diff = euler_angles_rad[k,j] - euler_angles_rad[k-1,j]
                if abs(diff) > np.pi:
                    euler_angles_rad[k:, j] -= np.sign(diff) * 2 * np.pi
        trajectory_rad[:, 3:] = euler_angles_rad
        
        # 转换为角度输出
        trajectory_deg = trajectory_rad.copy()
        trajectory_deg[:, 3:] = np.rad2deg(trajectory_rad[:, 3:])
        
        # 如果是中间段，去掉第一个点(避免重复)
        if i > 0:
            trajectory_deg = trajectory_deg[1:]
            t_values = t_values[1:]
        
        full_trajectory.append(trajectory_deg)
        segment_info.append({
            'points': trajectory_deg,
            't_values': t_values,
            'start': start,
            'end': end,
            'color': plt.cm.get_cmap(color_map)(i/(len(waypoints)-1))  # 为每段分配颜色
        })
    
    # 合并所有段
    final_trajectory = np.vstack(full_trajectory)
    
    # ===== 增强可视化 =====
    plt.figure(figsize=(18, 12))
    ax = plt.subplot(111, projection='3d')
    
    # 创建全局颜色映射
    global_t = np.linspace(0, 1, len(final_trajectory))
    global_colors = plt.cm.get_cmap(color_map)(global_t)
    
    # 1. 绘制全局轨迹线（带颜色渐变）
    for i in range(len(final_trajectory)-1):
        ax.plot(
            final_trajectory[i:i+2, 0], 
            final_trajectory[i:i+2, 1], 
            final_trajectory[i:i+2, 2],
            color=global_colors[i], linewidth=2, alpha=0.8
        )
    
    # 2. 显示所有中间点（带颜色渐变）
    if show_all_points:
        sc = ax.scatter(
            final_trajectory[:, 0], final_trajectory[:, 1], final_trajectory[:, 2],
            c=global_t, cmap=color_map, s=30, alpha=0.7,
            label='轨迹点'
        )
        plt.colorbar(sc, ax=ax, label='全局进度')
    
    # 3. 显示姿态坐标系（确保每orientation_skip个点显示一个）
    if show_orientation:
        def plot_orientation_frame(ax, position, euler_deg, color, scale=0.3, alpha=1.0):
            """绘制一个姿态坐标系（输入角度单位）"""
            rot = R.from_euler(angle_type, np.deg2rad(euler_deg))
            axis_len = scale
            x_axis = rot.apply([axis_len, 0, 0])
            y_axis = rot.apply([0, axis_len, 0])
            z_axis = rot.apply([0, 0, axis_len])
            
            ax.quiver(
                *position, *x_axis, color='r', 
                arrow_length_ratio=0.2, linewidth=1.5, alpha=alpha
            )
            ax.quiver(
                *position, *y_axis, color='g', 
                arrow_length_ratio=0.2, linewidth=1.5, alpha=alpha
            )
            ax.quiver(
                *position, *z_axis, color='b', 
                arrow_length_ratio=0.2, linewidth=1.5, alpha=alpha
            )
        
        # 绘制关键姿态坐标系
        for i in range(0, len(final_trajectory), orientation_skip):
            plot_orientation_frame(
                ax, 
                final_trajectory[i, :3], 
                final_trajectory[i, 3:],
                global_colors[i], 
                scale=10, 
                alpha=0.8
            )
    
    # 标记所有路径点
    for i, wp in enumerate(waypoints):
        color = 'red' if i == 0 else ('green' if i == len(waypoints)-1 else 'purple')
        ax.scatter(*wp[:3], c=color, s=150, marker='o', edgecolors='k', linewidths=1.5)
        ax.text(*wp[:3], f'WP{i}\n({wp[0]:.1f},{wp[1]:.1f},{wp[2]:.1f})', 
                color=color, fontsize=9, ha='center')
        
        # 绘制路径点的姿态
        if show_orientation:
            plot_orientation_frame(ax, wp[:3], wp[3:], color, scale=10)
    
    # 设置坐标轴属性
    ax.set_xlabel('X (m)', fontsize=12)
    ax.set_ylabel('Y (m)', fontsize=12)
    ax.set_zlabel('Z (m)', fontsize=12)
    ax.set_title('多段六维笛卡尔坐标轨迹\n(姿态坐标系每{}个点显示一个)'.format(orientation_skip), fontsize=14)
    
    # 等比例缩放
    all_points = np.vstack([wp[:3] for wp in waypoints] + [final_trajectory[:, :3]])
    max_range = max(
        np.ptp(all_points[:, 0]), 
        np.ptp(all_points[:, 1]), 
        np.ptp(all_points[:, 2])
    ) / 2.0
    mid_x = np.mean(all_points[:, 0])
    mid_y = np.mean(all_points[:, 1])
    mid_z = np.mean(all_points[:, 2])
    
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)
    
    # 添加图例
    # from matplotlib.patches import Patch
    # legend_elements = [
    #     Patch(facecolor='red', label='起点'),
    #     Patch(facecolor='green', label='终点'),
    #     Patch(facecolor='purple', label='中间点'),
    #     Patch(facecolor='blue', label='轨迹点'),
    #     Patch(facecolor='white', label=f'姿态坐标系(每{orientation_skip}个点)')
    # ]
    # ax.legend(handles=legend_elements, loc='upper right')
    
    # ===== 姿态角度变化图 =====
    # plt.figure(figsize=(15, 8))
    # angle_names = {
    #     'zyx': ['偏航 (Z)', '俯仰 (Y)', '横滚 (X)'],
    #     'xyz': ['横滚 (X)', '俯仰 (Y)', '偏航 (Z)']
    # }.get(angle_type, ['A', 'B', 'C'])
    
    # 计算全局t值
    # global_t = np.linspace(0, 1, len(final_trajectory))
    
    # for i, name in enumerate(angle_names):
    #     plt.plot(global_t, final_trajectory[:, 3+i], '-', 
    #             linewidth=2.5, alpha=0.8,
    #             label=f"{name}: {waypoints[0][3+i]:.1f}° → {waypoints[-1][3+i]:.1f}°")
    
    # 标记段边界
    # cum_points = 0
    # for idx, seg in enumerate(segment_info):
    #     cum_points += len(seg['points'])
    #     t_pos = cum_points / len(final_trajectory)
    #     if idx < len(segment_info)-1:
    #         plt.axvline(x=t_pos, color='gray', linestyle='--', alpha=0.7)
    #         plt.text(t_pos, plt.ylim()[1]*0.95, f'段{idx+2}', 
    #                 ha='center', va='top', backgroundcolor='w',
    #                 bbox=dict(facecolor='white', alpha=0.8, edgecolor='gray'))
    
    plt.xlabel('全局进度参数 t', fontsize=12)
    plt.ylabel('角度 (度)', fontsize=12)
    plt.title('欧拉角变化过程 (单位:度)', fontsize=14)
    plt.legend(fontsize=10, loc='upper right')
    plt.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()
    
    return final_trajectory

# 示例使用（修复可视化问题）
if __name__ == "__main__":
    # 定义路径点序列（角度单位）
    waypoints = [
        [100,200,100,100,60,100],    # 起点
        [203, 336, 218, -79, 67, -101],    # 终点1
        [200, 344, 96, 113, 60, 91] ,    # 终点3
        [174, 405, 50, -174, 87, 162]  # 终点2

    ]

    trajectory = generate_multi_point_6d_trajectory_enhanced(
        waypoints, 
        nums=[10, 10, 10],  # 每段的中间点数
        show_all_points=True,
        show_orientation=True,
        orientation_skip=2,  # 确保这个参数生效
        color_map='plasma'   # 使用新的颜色图谱
    )
    
    # 打印轨迹信息
    print(f"总轨迹点数: {len(trajectory)}")
    print("起点:", trajectory)
    # print("第一转折点:", trajectory[30])  # 第一段有30个中间点
    # print("第二转折点:", trajectory[30+40])
    # print("终点:", trajectory[-1])