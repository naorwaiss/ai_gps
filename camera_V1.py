import cv2
import numpy as np

class VideoStreamer:
    def __init__(self, camera_index=0,buffer_size= 5):
        self.camera_index = camera_index
        self.cap = cv2.VideoCapture(self.camera_index)
        self.data_save = buffer_size
        self.buffer = [None]*self.data_save


        # Check if the camera opened successfully
        if not self.cap.isOpened():
            print("Error: Unable to open camera.")
            return

    def start_stream(self):
        while True:
            # Read a frame from the camera
            ret, frame = self.cap.read()

            # Check if the frame was read successfully
            if not ret:
                print("Error: Unable to read frame.")
                break

            # Display the frame
            cv2.imshow('Video Stream', frame)

            # Print the frame (this will print the pixel values)
            frame_np = np.array(frame)
            red_channel = frame_np[:,:,0]
            green_channel = frame_np[:, :, 0]
            blue_channel = frame_np[:, :, 0]




            # Exit loop if 'q' is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    def stop_stream(self):
        # Release the camera and close the OpenCV windows
        self.cap.release()
        cv2.destroyAllWindows()



    #def data_save(self):

    #def frame_proccesin(self):


def main():
    # Create a VideoStreamer object with default camera index
    streamer = VideoStreamer()

    streamer.start_stream()
    # Stop streaming and release resources
    streamer.stop_stream()


if __name__ == "__main__":
    main()
