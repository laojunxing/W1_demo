import numpy as np

def generate_cuboid_ply(length, width, height, filename):
    # 定义长方体的8个顶点坐标 (mm)
    vertices = np.array([
        [0, 0, 0],          # 0
        [length, 0, 0],     # 1
        [length, width, 0], # 2
        [0, width, 0],      # 3
        [0, 0, height],    # 4
        [length, 0, height], # 5
        [length, width, height], # 6
        [0, width, height]  # 7
    ], dtype=np.float32)
    
    # 定义12个三角形面（每个四边形面分成2个三角形）
    faces = np.array([
        # 底面 (2个三角形)
        [0, 1, 2],
        [0, 2, 3],
        # 顶面 (2个三角形)
        [4, 6, 5],
        [4, 7, 6],
        # 前面 (2个三角形)
        [0, 5, 1],
        [0, 4, 5],
        # 后面 (2个三角形)
        [3, 2, 6],
        [3, 6, 7],
        # 右面 (2个三角形)
        [1, 5, 6],
        [1, 6, 2],
        # 左面 (2个三角形)
        [0, 3, 7],
        [0, 7, 4]
    ], dtype=np.int32)
    
    # 写入PLY文件
    with open(filename, 'w') as f:
        # 写入PLY头
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write(f"element vertex {len(vertices)}\n")
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        f.write(f"element face {len(faces)}\n")
        f.write("property list uchar int vertex_index\n")
        f.write("end_header\n")
        
        # 写入顶点数据
        for vertex in vertices:
            f.write(f"{vertex[0]} {vertex[1]} {vertex[2]}\n")
        
        # 写入面数据 (每个面是三角形)
        for face in faces:
            f.write(f"3 {face[0]} {face[1]} {face[2]}\n")

# 参数设置
length = 10.0  # mm
width = 10.0   # mm
height = 40.0  # mm
output_filename = "/home/dexforce/文档/cuboid_10x10x40.ply"

# 生成PLY文件
generate_cuboid_ply(length, width, height, output_filename)

print(f"长方体PLY文件已生成并保存到: {output_filename}")