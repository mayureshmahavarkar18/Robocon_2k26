import numpy as np
import cv2

print("Step 1: Calibration Shuru (Manual 4-Point Method)...")

# --- INPUT 1: WORLD COORDINATES (Aapko yahan input dena hai) ---
# (Aapke zameen ke (X, Y) points in mm, jo aapne measuring tape se naape hain)
# (Yeh 4 points usi plane par hone chahiye jise aap test kar rahe ho)
world_points = np.float32([
    [2.68, 0.26],  # P1 (e.g., Top-left) in mm
    [2.68, 0.97],  # P2 (e.g., Top-right) in mm
    [1.87, 0.26],  # P3 (e.g., Bottom-left) in mm
    [1.87, 0.97]   # P4 (e.g., Bottom-right) in mm
])

# --- INPUT 2: PIXEL COORDINATES (Aapko yahan input dena hai) ---
# (Ussi order mein, (u, v) points jo aapne 'calibration_image.jpg' se dhoondhe hain)
# (Aap yeh MS Paint ya GIMP se dhoondh sakte ho)
pixel_points = np.float32([
    [973, 697],  # p1
    [1272, 704],  # p2
    [946,843],  # p3
    [1383,839]   # p4
])

print("Inputs set ho gaye. H matrix calculate kar raha hoon...")

# 3. H MATRIX CALCULATE KARO
# Yeh function 2D-to-3D mapping ka "secret key" banata hai
try:
    H = cv2.getPerspectiveTransform(pixel_points, world_points)
    
    # 4. MATRIX KO SAVE KARO
    # Is matrix ko 'homography.npy' file mein save kar rahe hain taaki main_test.py ise use kar sake
    np.save('homography.npy', H)

    print("Calibration Poora Hua!")
    print("Aapka H matrix hai:")

    print(H)
    print("Matrix ko 'homography.npy' mein save kar diya hai.")

except cv2.error as e:
    print(f"ERROR: Calibration fail ho gaya: {e}")
    print("Check karo ki aapke points sahi hain aur koi 3 points ek line mein nahi hain.")


    