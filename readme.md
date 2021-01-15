# 和rosproject仓库配合进行双目估计深度和发布点云
- rosbag play tf.bag
- rosrun stereo_ros stereo_ros_sub //双目估计深度及发布计算得到的稠密点云
- rosrun sparse_ros sparse_ros_pub //发布读取的点云文件
- rosrun my_topic_tools delay /camera/rgb/image_raw  //接收/camera/rgb/image_raw，整体延时并原样发布到/camera/rgb/image_raw_relay