import math

def calculate_angles(AB, BC, CD, DA, AC, BD):
    BA = AB
    CB = BC
    DC = CD
    AD = DA
    CA = AC
    DB = BD

    # 计算每个角的余弦值
    cos_A = (DB**2 - AD**2 - AB**2) / (2 * AD * AB)
    cos_B = (AC**2 - BC**2 - AB**2) / (2 * AB * BC)
    cos_C = (BD**2 - CD**2 - BC**2) / (2 * BC * CD)
    cos_D = (AC**2 - DC**2 - AD**2) / (2 * AD * DC)

    # 计算每个角的度数
    angle_A = math.degrees(math.acos(cos_A))
    angle_B = math.degrees(math.acos(cos_B))
    angle_C = math.degrees(math.acos(cos_C))
    angle_D = math.degrees(math.acos(cos_D))

    print(f"内角之和: {angle_A + angle_B + angle_C + angle_D:.2f}°")

    return angle_A, angle_B, angle_C, angle_D

# 输入四边形的四条边和两条对角线的长度
AB = float(input("请输入边AB的长度: "))
BC = float(input("请输入边BC的长度: "))
CD = float(input("请输入边CD的长度: "))
DA = float(input("请输入边DA的长度: "))
AC = float(input("请输入对角线AC的长度: "))
BD = float(input("请输入对角线BD的长度: "))

# 计算并输出每个角的度数
angles = calculate_angles(AB, BC, CD, DA, AC, BD)
print(f"角A的度数: {angles[0]:.2f}°")
print(f"角B的度数: {angles[1]:.2f}°")
print(f"角C的度数: {angles[2]:.2f}°")
print(f"角D的度数: {angles[3]:.2f}°")

fov = angles[0] + angles[1] - 180
print(f"FOV的度数: {fov:.2f}°")
