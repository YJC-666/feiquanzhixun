#!/usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import CompressedImage

def main():
    rospy.init_node('camera_node', anonymous=False)

    device_id = rospy.get_param('~device_id', 0)
    width     = rospy.get_param('~width', 640)
    height    = rospy.get_param('~height', 480)
    frame_id  = rospy.get_param('~frame_id', 'camera')

    cap = cv2.VideoCapture(device_id, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    actual_w = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    actual_h = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    if not cap.isOpened():
        rospy.logerr("Failed to open /dev/video%d", device_id)
        return

    rospy.loginfo("Camera /dev/video%d: %.0fx%.0f", device_id, actual_w, actual_h)

    pub = rospy.Publisher('/camera/image/compressed', CompressedImage, queue_size=1)
    seq = 0

    while not rospy.is_shutdown():
        ok, frame = cap.read()
        if not ok or frame is None:
            rospy.logwarn_throttle(2, "Camera read failed")
            rospy.sleep(0.01)
            continue

        ok, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
        if not ok:
            continue

        msg = CompressedImage()
        msg.header.stamp    = rospy.Time.now()
        msg.header.frame_id = frame_id
        msg.header.seq      = seq
        msg.format          = 'jpeg'
        msg.data            = jpeg.tobytes()

        pub.publish(msg)
        seq += 1

    cap.release()

if __name__ == '__main__':
    main()