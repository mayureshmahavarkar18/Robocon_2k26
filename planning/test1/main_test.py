import numpy as np
from perception import find_ball_pixel  # perception.py se function import
from mapping import map_pixel_to_world    # mapping.py se function import

# --- INPUT 1: Ball waali image ka naam (Aapko yahan input dena hai) ---
# (Yeh photo aapne ball ko zameen par rakh kar li hai)
TEST_IMAGE_FILE = r"C:\Users\lenovo\Downloads\ball_test_image4.jpg"

# --- INPUT 2: Ball ki sample photo ka naam (Aapko yahan input dena hai) ---
# (Yeh aapko ball ki crop ki hui photo banani hai)

# --- INPUT 3: Ball ka asli coordinate (ANSWER KEY) (Aapko yahan input dena hai) ---
# (Yeh aapne measuring tape se naapa hai, mm mein)
REAL_X = 3.75


  # (e.g., 300mm)
REAL_Y = 0.75  # (e.g., 400mm)

# --- INPUT 4: Kitni galti (error) allowed hai (Aapko yahan input dena hai) ---
ALLOWED_ERROR_MM = 0.5 # (e.g., 20mm ki galti allowed hai)

print("--- POORA MAPPING TEST SHURU (Template Matching) ---")

# 1. 'H' matrix ko file se load karo
try:
    H = np.load('homography.npy')
    print("H matrix file ('homography.npy') load ho gayi.")
except FileNotFoundError:
    print("FATAL ERROR: 'homography.npy' file nahi mili. Pehle calibration.py chalao.")
    exit()

# 2. "Eyes": Ball ka pixel (u, v) dhoondho
# Yeh perception.py ko call kar raha hai
pixel_coord = find_ball_pixel(TEST_IMAGE_FILE)

# Check karo ki ball mili ya nahi
if pixel_coord:
    u_ball, v_ball = pixel_coord
    
    # 3. "Brain": (u, v) ko (X, Y) mein badlo
    # Yeh mapping.py ko call kar raha hai
    world_coord = map_pixel_to_world(H, u_ball, v_ball)
    
    if world_coord:
        X_calc, Y_calc = world_coord
        
        # 4. Result Check Karo
        error_X = abs(X_calc - REAL_X)
        error_Y = abs(Y_calc - REAL_Y)
        # --- ERROR PERCENTAGE CALCULATION ---
        # (Zero division se bachne ke liye check)
        if REAL_X == 0:
            error_perc_X = float('inf') if error_X > 0 else 0
        else:
            error_perc_X = (error_X / abs(REAL_X)) * 100

        if REAL_Y == 0:
            error_perc_Y = float('inf') if error_Y > 0 else 0
        else:
            error_perc_Y = (error_Y / abs(REAL_Y)) * 100
        
        print("\n--- FINAL VERIFICATION ---")
        print(f"real Coords: X={REAL_X} mm, Y={REAL_Y} mm")
        print(f" Calculated Coords: X={X_calc:.2f} mm, Y={Y_calc:.2f} mm")
        print(f"Error: X me {error_X:.2f} mm, Y me {error_Y:.2f} mm")
        print(f"Error Percentage : X me {error_perc_X:.2f} %, Y me {error_perc_Y:.2f} %") # <-- NAYI LINE
        
        # Check karo ki galti allowed limit ke andar hai ya nahi
        if error_X < ALLOWED_ERROR_MM and error_Y < ALLOWED_ERROR_MM: 
            print("\nRESULT: PASS! 🟢 Calculations VALID hain.")
        else:
            print(f"\nRESULT: FAIL! 🔴 Galti {ALLOWED_ERROR_MM}mm se zyaada hai. Calibration check karo.")
    else:
        print("TEST FAILED: 🔴 Mapping calculation fail ho gayi.")
else:
    print("TEST FAILED: 🔴 Ball hi nahi mili image mein. Template/Threshold check karo.")