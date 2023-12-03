import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from cv_bridge import CvBridge

import cv2
import numpy as np
import joblib


# Adjust these values based on sign colors
lower_color = np.array([0, 100, 40])
upper_color = np.array([180, 255, 191])

# load the model from disk
loaded_model = joblib.load('/home/ztung/r2_ws/src/bb8_final/bb8_final/knn_model_1200.sav')


class KNN(Node):
    def __init__(self):
        super().__init__('knn_model')

        image_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
		    history=QoSHistoryPolicy.KEEP_LAST,
		    durability=QoSDurabilityPolicy.VOLATILE,
		    depth=1
		)

        # Declare that the image_subscriber node is subscribing to the /camera/image/compressed topic
        self._image_subscriber = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self._image_callback,
            image_qos_profile)
        self._image_subscriber  # Prevent unused variable warning

        # Publish the label of the sign
        self.sign_publisher = self.create_publisher(String, '/sign/label', 10)
        self.sign = String()

        self.timer = self.create_timer(0.5, self.timer_callback)



    # Define the new crop function that uses color thresholding
    def find_sign_by_color(self, img_hsv, lower_color, upper_color):
        mask = cv2.inRange(img_hsv, lower_color, upper_color)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        min_area = 825

        if not contours or cv2.contourArea(max(contours, key=cv2.contourArea)) < min_area:
            height, width = img_hsv.shape[:2]
            size = min(height, width) // 2
            center = (height // 2, width // 2)
            return img_hsv[center[0] - size // 2:center[0] + size // 2,
                        center[1] - size // 2:center[1] + size // 2]

        largest_contour = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largest_contour)
        cropped_img = img_hsv[y:y+h, x:x+w]
        return cropped_img
    

    def _image_callback(self, CompressedImage):
        # The "CompressedImage" is transformed to a color image in BGR space and is store in "_imgBGR"
        self._imgBGR = CvBridge().compressed_imgmsg_to_cv2(CompressedImage, "bgr8")
        
        # Image Processing to use the color thresholding
        img_hsv = cv2.cvtColor(self._imgBGR, cv2.COLOR_BGR2HSV)

        processed_images = []
        cropped_img = self.find_sign_by_color(img_hsv, lower_color, upper_color)
        if cropped_img is not None and cropped_img.size > 0:
            resized_img = cv2.resize(cropped_img, (33, 25))
            processed_images.append(resized_img)
        else:
            print(f"Sign not found in image")
            resized_img = np.zeros((33, 25, 3), dtype=np.uint8)  # Placeholder for images without a detected sign
            processed_images.append(resized_img)

        # Process test images for prediction
        test_images_processed = np.array(processed_images)
        # Flatten the processed test images
        test_data = test_images_processed.reshape(test_images_processed.shape[0], -1)

        # Predict labels using the loaded model
        loaded_predictions = loaded_model.predict(test_data)
        print("predicted label: ", loaded_predictions)

        if loaded_predictions[0] == 1:
            print('Turn Left!')
            self.sign.data = 'left'
        elif loaded_predictions[0] == 2:
            print('Turn Right!')
            self.sign.data = 'right'
        elif loaded_predictions[0] == 3:
            print('Reverse')
            self.sign.data = 'reverse'
        elif loaded_predictions[0] == 4:
            print('Reverse!')
            self.sign.data = 'reverse'
        elif loaded_predictions[0] == 5:
            print('Stop!')
            self.sign.data = 'stop'
        else:
            print('Nothing!')
            self.sign.data = 'empty'


    def timer_callback(self):
        # Publish the predicted label
        self.sign_publisher.publish(self.sign)


def main(args=None):
    rclpy.init(args=args)
    knn_model = KNN()
    rclpy.spin(knn_model)
    knn_model.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()