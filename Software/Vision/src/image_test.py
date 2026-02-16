import cv2
import numpy as np

def find_center(image_path):

    image = cv2.imread(image_path)
    if image is None:
        print("Can't read image")
        return None

    # Down scale for optimisation(50%)
    scale = 0.5
    small = cv2.resize(image, None, fx=scale, fy=scale)

    gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)

    # small blur
    blurred = cv2.GaussianBlur(gray, (5,5), 1.5)

    # automatic threshold
    _, thresh = cv2.threshold(
        blurred, 0, 255,
        cv2.THRESH_BINARY + cv2.THRESH_OTSU
    )

    # cleaning
    kernel = np.ones((5,5), np.uint8)
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)

    # find countour
    contours, _ = cv2.findContours(
        thresh,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    if not contours:
        print("not contour found")
        return None

    # take the biggest countour
    largest = max(contours, key=cv2.contourArea)

    # smallest circle that still can be around the target
    (x, y), radius = cv2.minEnclosingCircle(largest)

    # put back to scale
    x = int(x / scale)
    y = int(y / scale)

    center = (x, y)

    print("Center:", center)

    output = image.copy()
    cv2.circle(output, center, 3, (0, 0, 255), -1)
    display = cv2.resize(output, None, fx=0.5, fy=0.5)

    cv2.imshow("Detection", display)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    return center


def focus_score(image_path):

    image = cv2.imread(image_path)
    if image is None:
        print("Can't read image")
        return None

    # Downscale for performance
    scale = 0.5
    small = cv2.resize(image, None, fx=scale, fy=scale)

    gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)

    # masque to remove the black background
    _, mask = cv2.threshold(
        gray, 0, 255,
        cv2.THRESH_BINARY + cv2.THRESH_OTSU
    )

    masked = cv2.bitwise_and(gray, gray, mask=mask)

    # calculate the focus
    laplacian = cv2.Laplacian(masked, cv2.CV_64F)
    focus = laplacian.var()

    print(f"{image_path} Raw focus score: {focus:.2f}")


    return focus

if __name__ == "__main__":
    find_center("test_data/lune_2fev/moon_focus.PNG")
    find_center("test_data/lune_2fev/moon_not_focus.PNG")
    find_center("test_data/lune_2fev/moon_not_center.PNG")
    find_center("test_data/lune_2fev/moon_not_center1.PNG")

    focus_score("test_data/lune_2fev/moon_focus.PNG")
    focus_score("test_data/lune_2fev/moon_focus1.PNG")
    focus_score("test_data/lune_2fev/moon_focus2.PNG")
    focus_score("test_data/lune_2fev/moon_focus3.PNG")
    focus_score("test_data/lune_2fev/moon_half_focus.PNG")
    focus_score("test_data/lune_2fev/moon_not_focus.PNG")
    focus_score("test_data/lune_2fev/moon_not_focus1.PNG")
    focus_score("test_data/lune_2fev/moon_not_focus2.PNG")
    focus_score("test_data/lune_2fev/moon_not_focus3.PNG")