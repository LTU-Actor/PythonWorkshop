#! /usr/bin/env python3

from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
import rospy
import math

XMIN = 1
YMIN = -1
YMAX = 1
ZMIN = -5
ZMAX = 5

def callback_pointcloud(data):
    assert isinstance(data, PointCloud2)
    gen = point_cloud2.read_points(data, field_names=("x", "y", "z"),
                                   skip_nans=True)
    
    closest_object_dist = 100
    for p in gen:
        x = p[0]
        y = p[1]
        z = p[2]
        if( x > XMIN ):
           if( y > YMIN and y < YMAX ):
                if( z > ZMIN and z < ZMAX ):
                    dist = math.sqrt(x*x + y*y + z*z)
                    if( dist < closest_object_dist ):
                        closest_object_dist = dist

    print('Closest Obj = %8.3f' % closest_object_dist)

def main():
    rospy.init_node('pcl_listener', anonymous=True)
    rospy.Subscriber('/velodyne_points', PointCloud2, callback_pointcloud)
    rospy.spin()
    
if __name__ == "__main__":
    main()
                                                          
