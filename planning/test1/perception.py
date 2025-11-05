import cv2
import numpy as np

def find_ball_pixel(image_path):
    print(f"\nStep 2: '{r"C:\Users\lenovo\Downloads\ball_test_image4.jpg"}' mein purple ball (color) dhoondh raha hoon...")
    
    # Image load karo
    image = cv2.imread(image_path)
    if image is None:
        print(f"Error: Image '{r"C:\Users\lenovo\Downloads\ball_test_image4.jpg"}' load nahi hui. File name check karo.")
        return None

    # Image ko HSV color space mein convert karo (rang dhoondhne ke liye best)
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # --- INPUT 3: PURPLE RANG KI RANGE (Ise adjust karna pad sakta hai) ---
    # (Yeh purple ki ek common range hai. Aapko ise adjust karna pad sakta hai)
    # (H: Hue, S: Saturation, V: Value)
    lower_purple = np.array([125, 50, 50])
    upper_purple = np.array([155, 255, 255])
    
    # 2. MASK BANAO (Sirf purple cheezon ko white (safed) rakho)
    mask = cv2.inRange(hsv_image, lower_purple, upper_purple)
    
    # 3. CONTOURS (Boundaries) DHOONDHO
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) > 0:
        # Sabse bada contour (jo aapki ball honi chahiye) dhoondho
        c = max(contours, key=cv2.contourArea)
        
        # Thoda filter karo (bohot chhota noise ignore karo)
        if cv2.contourArea(c) < 100: # 100 pixel se chhota area ignore karo
             print("Error: Koi badi purple ball nahi mili (sirf noise mila).")
             return None
             
        # Uska center (u, v) nikaalo
        M = cv2.moments(c)
        if M["m00"] != 0:
            u = int(M["m10"] / M["m00"])
            v = int(M["m01"] / M["m00"])
            
            print(f"Ball mili! Pixel coordinate: u={u}, v={v}")
            return (u, v)
    
    print("Error: Koi purple ball nahi mili. (HSV range check karo)")
    return None