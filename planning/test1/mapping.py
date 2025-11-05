import numpy as np

# Yeh function main_test.py call karega
def map_pixel_to_world(H_matrix, u, v):
    print("\nStep 3: 2D se 3D coordinate calculate kar raha hoon...")

    # 1. PIXEL KO HOMOGENEOUS BANAO
    # (u, v) ko [u, v, 1] mein badlo taaki matrix se multiply kar sakein
    pixel_hom = np.array([u, v, 1], dtype=np.float32)

    # 2. MATRIX MULTIPLY KARO (H * p_hom)
    # Yahi hai woh main calculation jo aapke saare (Intrinsic/Extrinsic)
    # parameters ko use kar rahi hai jo H matrix mein chhupe hain
    try:
        temp_coords = np.dot(H_matrix, pixel_hom)
    except ValueError as e:
        print(f"ERROR: Matrix multiplication fail hua: {e}")
        print("Check karo ki 'homography.npy' file sahi hai.")
        return None

    # 3. 'w' SE DIVIDE KARO (Perspective se scale karne ke liye)
    # temp_coords [x', y', w] format mein hai
    w = temp_coords[2]
    
    if w == 0:
        print("ERROR: 'w' zero hai. Calculation fail.")
        return None
        
    # Asli X aur Y nikaalo
    X_world = temp_coords[0] / w
    Y_world = temp_coords[1] / w

    print(f"Calculation poori hui: X = {X_world:.2f} mm, Y = {Y_world:.2f} mm")
    return (X_world, Y_world)