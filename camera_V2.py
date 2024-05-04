import cv2
import numpy as np
import queue
import threading

class OpticalFlowTracker:
    def __init__(self, video_source=0):
        self.cap = cv2.VideoCapture(video_source)
        self.prev_frame = None
        self.prev_pts = None
        self.prev_x = 0
        self.prev_y = 0
        self.output_queue = queue.Queue()

        # Parameters for Lucas-Kanade optical flow
        self.lk_params = dict(winSize=(15, 15),
                              maxLevel=2,
                              criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

        # Parameters for smoothing
        self.alpha = 0.2  # Smoothing factor

        # Noise canceling threshold
        self.movement_threshold = 5.0

    def calculate_optical_flow(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray_blur = cv2.GaussianBlur(gray, (5, 5), 0)

        if self.prev_frame is not None and self.prev_pts is not None:
            next_pts, status, _ = cv2.calcOpticalFlowPyrLK(self.prev_frame, gray_blur, self.prev_pts, None, **self.lk_params)
            good_new = next_pts[status == 1]
            good_old = self.prev_pts[status == 1]

            if len(good_new) > 0 and len(good_old) > 0:
                movement = np.mean(good_new - good_old, axis=0)
                x_movement = movement[0]
                y_movement = movement[1]

                if self.prev_x != 0 and self.prev_y != 0:
                    x_movement = self.alpha * x_movement + (1 - self.alpha) * self.prev_x
                    y_movement = self.alpha * y_movement + (1 - self.alpha) * self.prev_y

                if abs(x_movement) < self.movement_threshold:
                    x_movement = 0
                if abs(y_movement) < self.movement_threshold:
                    y_movement = 0

                self.prev_x = x_movement
                self.prev_y = y_movement

                return x_movement, y_movement

        return 0, 0

    def run(self):
        while True:
            ret, frame = self.cap.read()
            if not ret:
                break

            movement = self.calculate_optical_flow(frame)
            self.output_queue.put(movement)

            self.prev_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            self.prev_pts = cv2.goodFeaturesToTrack(self.prev_frame, maxCorners=100, qualityLevel=0.01, minDistance=10)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.cap.release()
        cv2.destroyAllWindows()

def process_output(output_queue):
    while True:
        movement = output_queue.get()

if __name__ == "__main__":
    tracker = OpticalFlowTracker()
    output_thread = threading.Thread(target=process_output, args=(tracker.output_queue,))
    output_thread.start()
    tracker.run()
