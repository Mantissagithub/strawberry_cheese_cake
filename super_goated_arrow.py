import cv2
import numpy as np
import math

class EuclideanDistTracker:
    def __init__(self):
        self.center_points = {}
        self.id_count = 0

    def update(self, objects_rect):
        objects_bbs_ids = []
        for rect in objects_rect:
            x, y, w, h = rect
            cx = (x + x + w) // 2
            cy = (y + y + h) // 2

            same_object_detected = False
            for id, pt in self.center_points.items():
                dist = math.hypot(cx - pt[0], cy - pt[1])

                if dist < 20:
                    self.center_points[id] = (cx, cy)
                    objects_bbs_ids.append([x, y, w, h, id])
                    same_object_detected = True
                    break

            if not same_object_detected:
                self.center_points[self.id_count] = (cx, cy)
                objects_bbs_ids.append([x, y, w, h, self.id_count])
                self.id_count += 1

        new_center_points = {}
        for obj_bb_id in objects_bbs_ids:
            _, _, _, _, object_id = obj_bb_id
            center = self.center_points[object_id]
            new_center_points[object_id] = center

        self.center_points = new_center_points.copy()
        return objects_bbs_ids

def calculate_angles(approx):
    def angle(pt1, pt2, pt3):
        v1 = np.array(pt1) - np.array(pt2)
        v2 = np.array(pt3) - np.array(pt2)
        cosine_angle = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
        cosine_angle = np.clip(cosine_angle, -1.0, 1.0)
        return np.degrees(np.arccos(cosine_angle))

    angles = []
    for i in range(len(approx)):
        pt1 = approx[i - 1][0]
        pt2 = approx[i][0]
        pt3 = approx[(i + 1) % len(approx)][0]
        angles.append(angle(pt1, pt2, pt3))
    return angles

def calculate_brightness(roi):
    """
    Calculates the average brightness of a region of interest (ROI).
    
    Args:
        roi (numpy array): ROI in BGR format.

    Returns:
        float: Average brightness value (0-255).
    """
    # Convert the ROI to grayscale
    gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    # Calculate average brightness
    avg_brightness = np.mean(gray_roi)
    return avg_brightness

def calculate_global_maxima_ratio_split(image, brightness_threshold):
    """
    Calculates variance and mean ratio of histogram peaks based on brightness.
    
    Args:
        image (numpy array): ROI in BGR format.
        brightness_threshold (float): Threshold for brightness.

    Returns:
        tuple: Variance and mean ratio based on the histogram split logic.
    """
    ratios = {}
    hist_split = (100, 255) if brightness_threshold < 130 else (150, 255)

    for i, channel_name in enumerate(['Blue', 'Green', 'Red']):
        channel = image[:, :, i]
        hist = cv2.calcHist([channel], [0], None, [256], [0, 256]).flatten()

        hist_low = hist[:hist_split[0] + 1]
        hist_high = hist[hist_split[0] + 1:hist_split[1] + 1]

        global_max_low = max(hist_low)
        global_max_high = max(hist_high)

        ratio = global_max_low / global_max_high if global_max_high != 0 else 0
        ratios[channel_name] = ratio

    ratio_values = list(ratios.values())
    mean_ratio = np.mean(ratio_values)
    variance = np.var(ratio_values)

    return variance * 100, mean_ratio


# Open the webcam with higher resolution
cap = cv2.VideoCapture(2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

# Parameters for noise reduction and edge detection
median_blur_ksize = 7
gaussian_blur_ksize = (7, 7)
canny_threshold1 = 50
canny_threshold2 = 150

tracker = EuclideanDistTracker()

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Could not read frame from webcam.")
        break

    # Add a white border around the frame
    border_size = 20
    frame_with_border = cv2.copyMakeBorder(
        frame, border_size, border_size, border_size, border_size,
        cv2.BORDER_CONSTANT, value=[255, 255, 255]
    )

    # Preprocess the frame
    frame_denoised = cv2.medianBlur(frame_with_border, median_blur_ksize)
    frame_denoised = cv2.GaussianBlur(frame_denoised, gaussian_blur_ksize, 0)
    gray = cv2.cvtColor(frame_denoised, cv2.COLOR_BGR2GRAY)

    cv2.imshow("blurs",frame_denoised)
    cv2.imshow("gray",gray)
    # Apply adaptive thresholding and edge detection
    binary = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                   cv2.THRESH_BINARY_INV, 11, 2)
    edges = cv2.Canny(binary, canny_threshold1, canny_threshold2)

    # Find contours in the frame
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
     # Create a blank image to draw contours
    contours_display = np.zeros_like(frame_with_border)
    cv2.drawContours(contours_display, contours, -1, (0, 255, 0), 2)

    # Display the contours image
    cv2.imshow('Contours', contours_display)

    detected_arrows = []
    for contour in contours:
        if cv2.contourArea(contour) < 750:
            continue

        epsilon = 0.02 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)

        if len(approx) == 7:
            angles = calculate_angles(approx)
            angles.sort()
            if (20 <= angles[0] <= 80 and
                50 <= angles[3] <= 110 and
                50 <= angles[4] <= 110 and
                20 <= angles[1] <= 80 and
                20 <= angles[2] <= 80):

                x, y, w, h = cv2.boundingRect(contour)
                roi = frame_with_border[y:y + h, x:x + w]
                brightness = calculate_brightness(roi)
                variance, mean_ratio = calculate_global_maxima_ratio_split(roi, brightness)

                if variance > 0 and mean_ratio > 0 and variance < 1.5 and mean_ratio < 1:
                    detected_arrows.append((x, y, w, h))

    boxes_ids = tracker.update(detected_arrows)
    for box_id in boxes_ids:
        x, y, w, h, id = box_id
        cv2.rectangle(frame_with_border, (x, y), (x + w, y + h), (0, 0, 255), 10)
        cv2.putText(frame_with_border, str(id), (x, y - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

    cv2.imshow('Detected Arrows', frame_with_border)

    # Break the loop when 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the webcam and close all windows
cap.release()
cv2.destroyAllWindows()