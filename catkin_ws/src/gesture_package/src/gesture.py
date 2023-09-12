#!/usr/bin/env python

import cv2
import mediapipe as mp
import numpy as np
import time
import rospy
from std_msgs.msg import String


if __name__ == "__main__":
    pub = rospy.Publisher('gesture_detected', String, queue_size=10)
    rospy.init_node("gesture_recognition")
    rate = rospy.Rate(20)
    

    #hand gesture
    mp_hands = mp.solutions.hands
    hands = mp_hands.Hands()
    mp_draw = mp.solutions.drawing_utils

    finger_tips = [8, 12, 16, 20]
    thumb_tip = 4

    #head gesture
    mp_face_mesh = mp.solutions.face_mesh
    face_mesh = mp_face_mesh.FaceMesh(min_detection_confidence=0.5, min_tracking_confidence=0.5)
    mp_drawing = mp.solutions.drawing_utils
    drawing_spec = mp_drawing.DrawingSpec(thickness=1, circle_radius=1)

    deny = 0
    affirm = 0
    time_d = 0
    time_a = 0

    left = False
    right = False
    up = False
    down = False

    answer = ""

    cap = cv2.VideoCapture(0)

    while not rospy.is_shutdown():
        success, image = cap.read()

        # Flip the image horizontally for a later selfie-view display
        # Also convert the color space from BGR to RGB
        image = cv2.cvtColor(cv2.flip(image, 1), cv2.COLOR_BGR2RGB)
        h, w, c = image.shape
        result = hands.process(image) #take img as an input and process it
        start = time.time()

        # To improve performance
        image.flags.writeable = False
        
        # Get the result
        results = face_mesh.process(image)
        
        # To improve performance
        image.flags.writeable = True
        
        # Convert the color space from RGB to BGR
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        img_h, img_w, img_c = image.shape
        face_3d = []
        face_2d = []

        text = ""

        if result.multi_hand_landmarks:
            for hand_landmark in result.multi_hand_landmarks:
                lm_list = []
                for id, lm in enumerate(hand_landmark.landmark):
                    lm_list.append(lm)
                finger_fold_status = []
                for tip in finger_tips:
                    x, y = int(lm_list[tip].x * w), int(lm_list[tip].y * h)
                    # print(id, ":", x, y)
                    cv2.circle(image, (x, y), 15, (255, 0, 0), cv2.FILLED)

                    if lm_list[thumb_tip].x < lm_list[tip].x:
                        if lm_list[tip].x < lm_list[tip - 3].x:
                            cv2.circle(image, (x, y), 15, (0, 255, 0), cv2.FILLED)
                            finger_fold_status.append(True)
                        else:
                            finger_fold_status.append(False)
                    elif lm_list[thumb_tip].x > lm_list[tip].x:
                        if lm_list[tip - 3].x < lm_list[tip].x:
                            cv2.circle(image, (x, y), 15, (0, 255, 0), cv2.FILLED)
                            finger_fold_status.append(True)
                        else:
                            finger_fold_status.append(False)

                print(finger_fold_status)

                if all(finger_fold_status):
                    #affirm
                    if lm_list[thumb_tip].y < lm_list[thumb_tip - 1].y < lm_list[thumb_tip - 2].y:
                        answer = "Yes"
                        cv2.putText(image, "Yes", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 3)
                        
                    #deny
                    if lm_list[thumb_tip].y > lm_list[thumb_tip - 1].y > lm_list[thumb_tip - 2].y:
                        answer = "No"
                        cv2.putText(image, "No", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)
                        
                mp_draw.draw_landmarks(image, hand_landmark,
                                    mp_hands.HAND_CONNECTIONS,
                                    mp_draw.DrawingSpec((0, 0, 255), 6, 3),
                                    mp_draw.DrawingSpec((0, 255, 0), 4, 2)
                                    )

        else:
            if results.multi_face_landmarks:
                for face_landmarks in results.multi_face_landmarks:
                    for idx, lm in enumerate(face_landmarks.landmark):
                        if idx == 33 or idx == 263 or idx == 1 or idx == 61 or idx == 291 or idx == 199:
                            if idx == 1:
                                nose_2d = (lm.x * img_w, lm.y * img_h)
                                nose_3d = (lm.x * img_w, lm.y * img_h, lm.z * 3000)

                            x, y = int(lm.x * img_w), int(lm.y * img_h)

                            # Get the 2D Coordinates
                            face_2d.append([x, y])

                            # Get the 3D Coordinates
                            face_3d.append([x, y, lm.z])       
                    
                    # Convert it to the NumPy array
                    face_2d = np.array(face_2d, dtype=np.float64)

                    # Convert it to the NumPy array
                    face_3d = np.array(face_3d, dtype=np.float64)

                    # The camera matrix
                    focal_length = 1 * img_w

                    cam_matrix = np.array([ [focal_length, 0, img_h / 2],
                                            [0, focal_length, img_w / 2],
                                            [0, 0, 1]])

                    # The distortion parameters
                    dist_matrix = np.zeros((4, 1), dtype=np.float64)

                    # Solve PnP
                    success, rot_vec, trans_vec = cv2.solvePnP(face_3d, face_2d, cam_matrix, dist_matrix)

                    # Get rotational matrix
                    rmat, jac = cv2.Rodrigues(rot_vec)

                    # Get angles
                    angles, mtxR, mtxQ, Qx, Qy, Qz = cv2.RQDecomp3x3(rmat)

                    # Get the y rotation degree
                    x = angles[0] * 360
                    y = angles[1] * 360
                    z = angles[2] * 360
                
                    # See where the user's head tilting
                    if y < -10 and x < 10:
                        left = True
                        time_d += 1
                        time_a = 0
                    elif y > 10 and x < 10:
                        right = True
                        time_d += 1
                        time_a = 0
                    elif x < -2 and y < 5:
                        down = True
                        time_a += 1
                        time_d = 0
                    elif x > 10 and y > 0:
                        up = True
                        time_a += 1
                        time_d = 0

                    #check if it is nodding or shaking
                    if left and right:
                        answer = "No"
                        text = "Deny"
                        deny += 1
                    elif time_d > 5 or left and not right and time_d > 5:
                        left = False
                        time_d = 0
                    elif time_d > 5 or right and not left and time_d > 5:
                        right = False
                        time_d = 0
                    elif up and down:
                        answer = "Yes"
                        text = "Affirm"
                        affirm += 1
                    elif up and not down and time_a > 5 or time_a > 5:
                        up = False
                        time_a = 0
                    elif down and not up and time_a > 5 or time_a > 5:
                        down = False
                        time_a = 0

                    # after deny/affirm goes above threshold sets it back to 0
                    if deny > 50:
                        left = False
                        right = False
                        deny = 0
                    elif affirm > 50:
                        up = False
                        down = False
                        affirm = 0

                    # Display the nose direction
                    nose_3d_projection, jacobian = cv2.projectPoints(nose_3d, rot_vec, trans_vec, cam_matrix, dist_matrix)

                    p1 = (int(nose_2d[0]), int(nose_2d[1]))
                    p2 = (int(nose_2d[0] + y * 10) , int(nose_2d[1] - x * 10))
                    
                    cv2.line(image, p1, p2, (255, 0, 0), 3)

                    # Add the text on the image
                    cv2.putText(image, text, (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 2)
                    cv2.putText(image, "x: " + str(np.round(x,2)), (500, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                    cv2.putText(image, "y: " + str(np.round(y,2)), (500, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                    cv2.putText(image, "z: " + str(np.round(z,2)), (500, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

                print([left, right, deny], [up, down,affirm], time_d, time_a)

                end = time.time()
                totalTime = end - start

                fps = 1 / totalTime
                #print("FPS: ", fps)

                cv2.putText(image, f'FPS: {int(fps)}', (20,450), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0,255,0), 2)

                mp_drawing.draw_landmarks(
                            image=image,
                            landmark_list=face_landmarks,
                            connections=mp_face_mesh.FACEMESH_CONTOURS,
                            landmark_drawing_spec=drawing_spec,
                            connection_drawing_spec=drawing_spec)


        if answer is not None:
            pub.publish(answer)

        cv2.imshow('Gesture Recognition', image)

        if cv2.waitKey(5) & 0xFF == 27:
            break

cap.release()
