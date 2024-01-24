from PIL import Image

# 打开6张图片
image0 = Image.open("/home/lyn/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/aerial_robot_simulation/gazebo_model/models/Apriltag36_11_00000/materials/textures/tag36_11_00000.png")
# image1 = Image.open("/home/lyn/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/aerial_robot_simulation/gazebo_model/models/Apriltag36_11_00001/materials/textures/tag36_11_00001.png")
# image2 = Image.open("/home/lyn/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/aerial_robot_simulation/gazebo_model/models/Apriltag36_11_00002/materials/textures/tag36_11_00002.png")
# image3 = Image.open("/home/lyn/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/aerial_robot_simulation/gazebo_model/models/Apriltag36_11_00003/materials/textures/tag36_11_00003.png")
# image4 = Image.open("/home/lyn/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/aerial_robot_simulation/gazebo_model/models/Apriltag36_11_00004/materials/textures/tag36_11_00004.png")
# image5 = Image.open("/home/lyn/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/aerial_robot_simulation/gazebo_model/models/Apriltag36_11_00005/materials/textures/tag36_11_00005.png")


# 设定固定大小
#new_width, new_height = 713, 713
new_width, new_height = 300, 300
# 调整图片大小
image0_resized = image0.resize((new_width, new_height))
# image1_resized = image1.resize((new_width, new_height))
# image2_resized = image2.resize((new_width, new_height))
# image3_resized = image3.resize((new_width, new_height))
# image4_resized = image4.resize((new_width, new_height))
# image5_resized = image5.resize((200, 200))


# 创建一个新的图像
result_width = 2100
result_height = 2970
result_image = Image.new("RGB", (result_width, result_height),(255,255,255,0))

# 将图片粘贴到新图像上（你可以根据需要修改坐标）
result_image.paste(image0_resized, (100, 100))
#result_image.paste(image1_resized, (512, 627))
#result_image.paste(image2_resized, (1112, 627))
#result_image.paste(image3_resized, (512, 1867))
#result_image.paste(image4_resized, (1112, 1867))
#result_image.paste(image5_resized, (1512, 1384))


# 保存结果
result_image.save("Apriltag_000.jpg")
