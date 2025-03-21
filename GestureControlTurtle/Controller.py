import cv2
import mediapipe as mp
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class GestureController(Node):
    def __init__(self):
        super().__init__('Gesture_Control')

        # Publishers for turtle1 and turtle2 
        self.publisher_turtleR = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.publisher_turtleL = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)

        # Initialize MediaPipe hands
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles
        self.mp_hands = mp.solutions.hands
        self.hands = mp.solutions.hands.Hands()

        # Initialize webcam input
        self.cap = cv2.VideoCapture(0)

    def process_gestures(self):
        alpha = 0.2 # Transparency factor for overlaid images
        hf_center_x=130
        hf_center_y=360
        hf_right_shift=370
        while True:
            ret, frame = self.cap.read()
            if not ret:
                break

            frame = cv2.flip(frame, 1)
            overlay=frame.copy()

            image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = self.hands.process(image)
            #twist_command = Twist()
            HandNum=0
            # Add hand detection logic here
            # Example: If right hand detected, move right TurtleBot forward
            if results.multi_hand_landmarks:
                for hand_landmarks in results.multi_hand_landmarks:
                    # rl_value: 1 => right, 2 => left
                    rl_value = results.multi_handedness[HandNum].classification[0].index 
                    # Use hand position or gesture classification to set twist_right or twist_left
                    center_x, center_y = get_hand_center(hand_landmarks, frame.shape[1], frame.shape[0])

                    hand_center=(center_x, center_y)
                    bullseye_center=(hf_center_x+hf_right_shift*rl_value, hf_center_y)

                    gesture = recognize_gesture(hand_landmarks)

                    relative_pos=(hand_center[0]-bullseye_center[0],hand_center[1]-bullseye_center[1])
                    
                    twist_command = twist_calculation(gesture,relative_pos)
                    if rl_value==1:
                        self.publisher_turtleR.publish(twist_command)
                    else:
                        self.publisher_turtleL.publish(twist_command)
                    
                    ##############################
                    # Drawing on the camera view #
                    side_text="Right" if rl_value==1 else "Left"
                    textLocx=50 +300*rl_value
                    
                    self.mp_drawing.draw_landmarks(overlay, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)

                    cv2.putText(frame, side_text, (textLocx, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2, cv2.LINE_AA)
                    cv2.putText(frame, f"Gesture: {gesture}", (textLocx, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2, cv2.LINE_AA)
                    cv2.putText(frame, f"Center: ({center_x}, {center_y})", (textLocx, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2, cv2.LINE_AA)
                    
                    HandColor=(255*(1-rl_value), 0, 255*rl_value)

                    cv2.circle(overlay, hand_center, 20, HandColor, -1)
                    

                    # Small circle around center
                    cv2.circle(overlay, bullseye_center, 40 , HandColor, 5)

                    # Big circle around center
                    cv2.circle(overlay, bullseye_center, 100 , HandColor, 5)

                    cv2.line(overlay,bullseye_center,hand_center,HandColor,5)

                    # Following line overlays transparent rectangle 
                    #  over the image 

                    # If the other hand is 
                    HandNum=HandNum+1
                frame = cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0) 

            cv2.imshow('Gesture Control', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.cap.release()
        cv2.destroyAllWindows()


def main():
    print('Hi from GestureControlTurtle.')
    rclpy.init()
    mynode = GestureController()
    mynode.process_gestures()

    mynode.destroy_node()
    rclpy.shutdown()

def get_hand_center(landmarks, frame_width, frame_height):
    palmlandmarks=[0, 5, 17]
    # x_coords = [landmark.x * frame_width for landmark in landmarks.landmark]
    # y_coords = [landmark.y * frame_height for landmark in landmarks.landmark]

    x_coords = [landmarks.landmark[i].x * frame_width for i in palmlandmarks]
    y_coords = [landmarks.landmark[i].y * frame_height for i in palmlandmarks]

    return int(np.mean(x_coords)), int(np.mean(y_coords))

def recognize_gesture(landmarks):
    fingers = []
    tips = [4, 8, 12, 16, 20]
    
    # Thumb (different checking logic since it moves differently)
    handrootx=landmarks.landmark[0].x 
    thumbtipx=landmarks.landmark[tips[0]].x 
    thumbrootx=landmarks.landmark[tips[0]-2].x
    d_handroot_thumbroot=thumbrootx-handrootx
    d_thumbroot_thumbtip=thumbtipx-thumbrootx
    r_thumb= d_thumbroot_thumbtip/d_handroot_thumbroot
    if r_thumb>0.3:
    #if landmarks.landmark[tips[0]].x > landmarks.landmark[tips[0] - 2].x:
        fingers.append(1)  # Thumb extended
    else:
        fingers.append(0)
    
    # Other four fingers
    for i in range(1, 5):
        if landmarks.landmark[tips[i]].y < landmarks.landmark[tips[i] - 2].y:
            fingers.append(1)  # Finger extended
        else:
            fingers.append(0)
    
    # Gesture dictionary
    gestures = {
        (0, 0, 0, 0, 0): "Fist",
        (0, 1, 1, 0, 0): "Peace Sign",
        (1, 1, 1, 1, 1): "Open Palm",
        (0, 1, 0, 0, 0): "Pointing Up",
        (1, 0, 0, 0, 0): "Thumbs Up",
        (0, 1, 1, 1, 1): "Four Fingers",
        (0, 0, 1, 1, 1): "Three Fingers",
        (0, 0, 0, 1, 1): "Two Fingers"
    }
    return gestures.get(tuple(fingers), "Unknown Gesture")

def twist_calculation(gesture,relative_pos):
    twist_command=Twist()
    twist_command.linear.x = 0.0
    twist_command.linear.y = 0.0
    twist_command.angular.z = 0.0
    rx=relative_pos[0]/100.0
    ry=relative_pos[1]/100.0
    match gesture:
        case "left":
            twist_command.linear.x = 1.5
            twist_command.angular.z = 1.5
        case 'sleft':
            twist_command.angular.z = 1.5
        case 'right':
            twist_command.linear.x = 1.5
            twist_command.angular.z = -1.5
        case 'sright':
            twist_command.angular.z = -1.5
        case 'gright':
            twist_command.linear.y = -1.5
        case "Peace Sign": # Position mode
            twist_command.linear.x = rx
            twist_command.linear.y = -ry
        case "Fist": # Go straight
            twist_command.linear.x = rx
            twist_command.linear.y = -ry
        case "Open Palm": # Stop
            twist_command.linear.x = 0.0
    return twist_command

if __name__ == '__main__':
    main()


