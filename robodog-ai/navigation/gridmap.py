import numpy as np

# --- Ukuran Grid ---
GRID_ROWS = 20
GRID_COLS = 20

def create_grid():
   
    # Buat grid kosong dulu (semua 0)
    grid = np.zeros((GRID_ROWS, GRID_COLS), dtype=int)

    # Tempatkan obstacle secara manual
    # Format: grid[baris][kolom] = 1
    obstacles = [
        (1, 2), (1, 3), (1, 4), (2, 6), (3, 6), (4, 6), (5, 1), (5, 2), (5, 3), (6, 8), (7, 8),                  
        (3, 3), (3, 4), (7, 3), (7, 4), (7, 5), (8, 7), (13, 18), (14,18)                         
    ]

    for (r, c) in obstacles:
        grid[r][c] = 1  # Tandai sebagai obstacle

    return grid




def is_walkable(grid, row, col):
   
    # Cek apakah masih dalam batas grid
    if row < 0 or row >= GRID_ROWS:
        return False
    if col < 0 or col >= GRID_COLS:
        return False

    # Cek apakah bukan obstacle
    if grid[row][col] == 1:
        return False

    return True  # Sel aman untuk dilewati