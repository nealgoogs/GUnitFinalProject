import cv2
import os

def extract_frames(video_path, output_folder="frames"):
    # Create the frames folder if it does not exist
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    # Load video
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print("Error: Could not open video.")
        return

    fps = cap.get(cv2.CAP_PROP_FPS)
    frame_interval = int(fps)  # Capture 1 frame per second

    frame_count = 0
    saved_count = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Save 1 frame per second
        if frame_count % frame_interval == 0:
            filename = os.path.join(output_folder, f"frame_{saved_count:05d}.jpg")
            cv2.imwrite(filename, frame)
            print(f"Saved {filename}")
            saved_count += 1

        frame_count += 1

    cap.release()
    print("\nDone! Total frames saved:", saved_count)


if __name__ == "__main__":
    video_file = "input_video.mp4"   
    extract_frames(video_file)
