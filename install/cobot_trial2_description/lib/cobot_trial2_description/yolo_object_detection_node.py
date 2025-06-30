#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
from ultralytics import YOLO

class YOLOObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('yolo_object_detection_node')
        
        # Initialize YOLO model with absolute path
        model_path = "/home/lenovo/ultramotive_ws/src/cobot_trial2_description/best.pt"
        
        # Check if model file exists
        if not os.path.exists(model_path):
            self.get_logger().error(f"Model file not found at: {model_path}")
            self.get_logger().error("Please ensure best.pt is in the specified directory")
            return
        
        try:
            self.model = YOLO(model_path)
            self.get_logger().info(f"YOLO model loaded successfully from: {model_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to load YOLO model: {e}")
            return
        
        # Detection parameters
        self.confidence_threshold = 0.6
        self.line_width_pixels = 2
        
        # Initialize CV Bridge for converting ROS images to OpenCV format
        try:
            self.bridge = CvBridge()
            self.get_logger().info("CV Bridge initialized successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize CV Bridge: {e}")
            return
        
        # Subscribe to camera image topic
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        # Publisher for annotated images
        self.annotated_image_pub = self.create_publisher(
            Image,
            '/camera/annotated_image',
            10
        )
        
        self.get_logger().info("YOLO Object Detection Node started")
        self.get_logger().info("Subscribed to: /camera/image_raw")
        self.get_logger().info("Publishing annotated images to: /camera/annotated_image")

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Run YOLO inference
            results = self.model.predict(
                source=cv_image,
                show=False,  # Don't show window in ROS environment
                save=False,  # Don't save files automatically
                conf=self.confidence_threshold,
                line_width=self.line_width_pixels,
                verbose=False  # Reduce console output
            )
            
            # Process results and log detected objects
            detected_objects = []
            annotated_image = cv_image.copy()
            
            for r in results:
                if r.boxes is not None and len(r.boxes) > 0:
                    # Get the annotated image from YOLO
                    annotated_image = r.plot()
                    
                    # Extract detected object names
                    for i, cls_id in enumerate(r.boxes.cls):
                        class_name = self.model.names[int(cls_id)]
                        confidence = r.boxes.conf[i].item()
                        detected_objects.append((class_name, confidence))
            
            # Log detected objects
            if detected_objects:
                object_names = [obj[0] for obj in detected_objects]
                unique_objects = list(set(object_names))
                
                # Log each unique object type
                for obj_name in unique_objects:
                    count = object_names.count(obj_name)
                    max_conf = max([conf for name, conf in detected_objects if name == obj_name])
                    self.get_logger().info(f"Detected: {obj_name} (count: {count}, max_confidence: {max_conf:.2f})")
                
                # Log summary
                self.get_logger().info(f"Total objects detected: {len(detected_objects)} - Types: {', '.join(unique_objects)}")
            else:
                self.get_logger().info("No objects detected")
            
            # Publish annotated image
            try:
                annotated_msg = self.bridge.cv2_to_imgmsg(annotated_image, "bgr8")
                annotated_msg.header = msg.header  # Keep original timestamp
                self.annotated_image_pub.publish(annotated_msg)
            except Exception as e:
                self.get_logger().warn(f"Failed to publish annotated image: {e}")
                
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = YOLOObjectDetectionNode()
        # Only spin if model and cv_bridge loaded successfully
        if hasattr(node, 'model') and hasattr(node, 'bridge'):
            rclpy.spin(node)
        else:
            node.get_logger().error("Node initialization failed. Exiting...")
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()