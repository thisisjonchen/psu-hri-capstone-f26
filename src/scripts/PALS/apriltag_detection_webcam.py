import cv2
from pupil_apriltags import Detector

TAG_ZONES = {
    0: "PICKUP",
    1: "DROP_OFF",
    2: "BORDER"
}

# Create AprilTag detector
detector = Detector(
    families="tag36h11",
    nthreads=1,
    quad_decimate=1.0,
    quad_sigma=0.0,
    refine_edges=1,
    decode_sharpening=0.25,
    debug=0
)

# Open default webcam
camera = cv2.VideoCapture(0)

# Drone Camera
#camera_url = ("http://drone1.local:8080/stream?topic=/raspicam_node/image&type=mjpeg&quality=70")
#camera = cv2.VideoCapture(camera_url)

if not camera.isOpened():
    print("Could not open webcam.")
    exit()


while True:

    ret, frame = camera.read()

    if not ret:
        print("Could not read camera frame.")
        break

    # AprilTag detection expects a grayscale image
    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)

    # Detect tags
    detections = detector.detect(gray)

    for detection in detections:

        tag_id = detection.tag_id

        # Look up what the tag represents
        zone = TAG_ZONES.get(tag_id,"UNKNOWN")

        print("Detected Tag:",tag_id,"Zone:",zone)

        # Get corners of detected tag
        corners = detection.corners.astype(int)

        # Draw box around tag
        for i in range(4):

            point1 = tuple(corners[i])
            point2 = tuple(corners[(i + 1) % 4])

            cv2.line(frame,point1,point2,(0, 255, 0),2)

        # Put tag information on screen
        center = detection.center.astype(int)

        cv2.putText(frame,"Tag {}: {}".format(tag_id, zone),tuple(center),cv2.FONT_HERSHEY_SIMPLEX,0.6,(0, 0, 255),2)

    # Show webcam feed
    cv2.imshow("AprilTag Detection",frame)

    # Press Q to quit
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break


camera.release()
cv2.destroyAllWindows()