#!/usr/bin/env python2
import rospy
import tf2_ros

def main():
    rospy.init_node('simple_tf_listener')

    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    rospy.sleep(1.0)  # Allow time to fill buffer

    parent_frame = "camera_link"
    child_frame = "marker_pallet"

    try:
        transform = tf_buffer.lookup_transform(parent_frame, child_frame, rospy.Time(0), rospy.Duration(2.0))
        t = transform.transform.translation
        r = transform.transform.rotation

        print("Transform from '{}' to '{}':".format(parent_frame, child_frame))
        print("Translation: x={:.3f}, y={:.3f}, z={:.3f}".format(t.x, t.y, t.z))
        print("Rotation (quaternion): x={:.3f}, y={:.3f}, z={:.3f}, w={:.3f}".format(r.x, r.y, r.z, r.w))

    except Exception as e:
        rospy.logerr("Could not get transform: {}".format(e))

if __name__ == '__main__':
    main()
