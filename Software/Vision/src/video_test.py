import cv2
import numpy as np

def process_frame(frame, scale=0.5,frame_id = 0):

    original = frame.copy()

    # Downscale
    small = cv2.resize(frame, None, fx=scale, fy=scale)

    gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 1.5)

    # Threshold Otsu
    _, thresh = cv2.threshold(
        blurred, 0, 255,
        cv2.THRESH_BINARY + cv2.THRESH_OTSU
    )

    # Morphological cleaning
    kernel = np.ones((5, 5), np.uint8)
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)

    # Find contours
    contours, _ = cv2.findContours(
        thresh,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    center = None

    if contours:
        largest = max(contours, key=cv2.contourArea)
        (x, y), radius = cv2.minEnclosingCircle(largest)

        # Rescale coordinates
        x = int(x / scale)
        y = int(y / scale)
        center = (x, y)

        # Draw center
        cv2.circle(original, center, 5, (0, 0, 255), -1)

    # ---- Focus calculation ----
    # Mask to remove black background
    _, mask = cv2.threshold(
        gray, 0, 255,
        cv2.THRESH_BINARY + cv2.THRESH_OTSU
    )

    masked = cv2.bitwise_and(gray, gray, mask=mask)
    laplacian = cv2.Laplacian(masked, cv2.CV_64F)
    focus = laplacian.var()

    # ---- Overlay text bottom right ----
    h, w = original.shape[:2]
    # print the values to do graph in excel
    text_focus = f"Focus: {focus:.3f}"
    text_center = f"Center: {center}" if center else "Center: None"
    text_frame_id = f"Frame: {frame_id}"
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.6
    thickness = 2

    # Calculate text size
    (tw1, th1), _ = cv2.getTextSize(text_focus, font, font_scale, thickness)
    (tw2, th2), _ = cv2.getTextSize(text_center, font, font_scale, thickness)
    (tw3, th3), _ = cv2.getTextSize(text_frame_id, font, font_scale, thickness)

    x_text = w - max(tw1, tw2) - 10
    y_text1 = h - 50
    y_text2 = h - 30
    y_text3 = h - 10

    cv2.putText(original, text_focus, (x_text, y_text3),
                font, font_scale, (0, 255, 0), thickness)

    cv2.putText(original, text_center, (x_text, y_text2),
                font, font_scale, (0, 255, 0), thickness)

    cv2.putText(original, text_frame_id, (x_text, y_text1),
                font, font_scale, (0, 255, 0), thickness)
    return original, focus, center


def process_video(video_path):
    window_size = 5  # number of frame for average
    focus_buffer = []
    cap = cv2.VideoCapture(video_path)
    # Prepare to record video
    fps = cap.get(cv2.CAP_PROP_FPS)
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')

    out = cv2.VideoWriter(
        "output_jupiter_focus.mp4",
        fourcc,
        fps,
        (width, height)
    )


    if not cap.isOpened():
        print("Cannot open video")
        return
    frame_id = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame_id += 1

        output, focus, center = process_frame(frame,0.5,frame_id)
        out.write(output)
        # Dynamic average
        focus_buffer.append(focus)

        if len(focus_buffer) > window_size:
            focus_buffer.pop(0)

        smooth_focus = np.mean(focus_buffer)

        if center is not None:
            print(f"{frame_id},{smooth_focus:.3f},{center[0]},{center[1]}")
        else:
            print(f"{frame_id},{smooth_focus:.3f},0,0")

        display = cv2.resize(output, None, fx=0.5, fy=0.5)
        cv2.imshow("Jupiter Detection", display)

        if cv2.waitKey(30) & 0xFF == ord('q'):
            break

    cap.release()
    out.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    process_video("test_video_jupiter.mp4")