import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.animation as animation
import numpy as np
import time

# Import dari file kita sendiri
from navigation.gridmap import create_grid, GRID_ROWS, GRID_COLS
from navigation.astar import astar

 
# KONFIGURASI WARNA
 
COLOR_EMPTY     = "#F0F4F8"   # Abu muda  → sel kosong
COLOR_OBSTACLE  = "#2D3748"   # Gelap     → obstacle
COLOR_PATH      = "#63B3ED"   # Biru muda → jalur A*
COLOR_ROBOT     = "#48BB78"   # Hijau     → posisi robot
COLOR_GOAL      = "#FC8181"   # Merah     → posisi goal
COLOR_VISITED   = "#FBD38D"   # Kuning    → sel yang sudah dilewati robot
COLOR_GRID_LINE = "#CBD5E0"   # Abu       → garis grid

 
# STATE PROGRAM (variabel global yang bisa diakses semua fungsi)
 
grid = create_grid()        
robot_pos = [0, 0]             
goal_pos  = None               
current_path = []              
path_index = 0                 
robot_trail = []            
is_moving = False             


# FUNGSI MENGGAMBAR GRID


def draw_grid(ax):
    ax.clear()  # Hapus gambar sebelumnya

    # --- Gambar setiap sel ---
    for r in range(GRID_ROWS):
        for c in range(GRID_COLS):

            # Tentukan warna berdasarkan jenis sel
            if (r, c) == tuple(robot_pos):
                color = COLOR_ROBOT      
            elif goal_pos is not None and (r, c) == tuple(goal_pos):
                color = COLOR_GOAL       
            elif grid[r][c] == 1:
                color = COLOR_OBSTACLE   
            elif (r, c) in current_path:
                color = COLOR_PATH       
            elif (r, c) in robot_trail:
                color = COLOR_VISITED   
            else:
                color = COLOR_EMPTY     

            # Gambar kotak untuk sel ini
            rect = mpatches.FancyBboxPatch(
                (c + 0.05, GRID_ROWS - r - 1 + 0.05),  # Posisi (x, y)
                0.9, 0.9,                                # Lebar, tinggi
                boxstyle="round,pad=0.02",
                facecolor=color,
                edgecolor=COLOR_GRID_LINE,
                linewidth=0.8
            )
            ax.add_patch(rect)

            # --- Tambahkan label koordinat kecil di setiap sel ---
            ax.text(
                c + 0.5,
                GRID_ROWS - r - 1 + 0.15,
                f"({r},{c})",
                ha='center', va='bottom',
                fontsize=5, color="#A0AEC0"
            )

    # --- Gambar simbol robot (R) ---
    ax.text(
        robot_pos[1] + 0.5,
        GRID_ROWS - robot_pos[0] - 1 + 0.5,
        "🤖",
        ha='center', va='center',
        fontsize=14
    )

    # --- Gambar simbol goal (📦) hanya jika goal sudah dipilih ---
    if goal_pos is not None:
        ax.text(
            goal_pos[1] + 0.5,
            GRID_ROWS - goal_pos[0] - 1 + 0.5,
            "📦",
            ha='center', va='center',
            fontsize=14
        )

    # --- Batas axes ---
    ax.set_xlim(0, GRID_COLS)
    ax.set_ylim(0, GRID_ROWS)
    ax.set_aspect('equal')
    ax.axis('off')  # Sembunyikan sumbu

    # --- Judul dinamis ---
    status = "Bergerak..." if is_moving else ("Klik sel untuk menentukan Goal!" if goal_pos is None else "Klik sel untuk set Goal baru")
    path_len = len(current_path) - 1 if current_path else 0
    goal_display = "Belum dipilih" if goal_pos is None else tuple(goal_pos)
    ax.set_title(
        f"Robot Delivery Simulation\n"
        f"Robot: {tuple(robot_pos)}  |  Goal: {goal_display}  |  "
        f"Panjang jalur: {path_len} langkah\n"
        f"Status: {status}",
        fontsize=10, pad=10
    )

    # --- Legend ---
    legend_elements = [
        mpatches.Patch(facecolor=COLOR_ROBOT,    label='Robot'),
        mpatches.Patch(facecolor=COLOR_GOAL,     label='Goal'),
        mpatches.Patch(facecolor=COLOR_OBSTACLE, label='Obstacle'),
        mpatches.Patch(facecolor=COLOR_PATH,     label='Jalur A*'),
        mpatches.Patch(facecolor=COLOR_VISITED,  label='Jejak Robot'),
    ]
    ax.legend(
        handles=legend_elements,
        loc='upper left',
        bbox_to_anchor=(1.01, 1),
        fontsize=8,
        framealpha=0.8
    )



# FUNGSI MENGHITUNG JALUR BARU

def calculate_new_path():
    """
    Memanggil A* untuk menghitung jalur dari posisi robot ke goal saat ini.
    Mengupdate variabel global current_path dan path_index.
    """
    global current_path, path_index, robot_trail

    # Jika goal belum dipilih user, tidak perlu hitung jalur
    if goal_pos is None:
        print("Goal belum dipilih. Klik sel untuk menentukan goal.")
        return

    print(f"\n=== A* Pathfinding ===")
    print(f"Start : {tuple(robot_pos)}")
    print(f"Goal  : {tuple(goal_pos)}")

    # Panggil algoritma A* dari astar.py
    path = astar(grid, tuple(robot_pos), tuple(goal_pos))

    if path:
        current_path = path
        path_index = 0            # Reset ke awal jalur
        robot_trail = []          # Reset jejak
        print(f"Jalur ditemukan: {path}")
        print(f"Panjang: {len(path) - 1} langkah")
    else:
        current_path = []
        print("Tidak ada jalur!")



# FUNGSI HANDLER KLIK MOUSE

def on_click(event):
    global goal_pos, is_moving

    # Abaikan klik jika robot sedang bergerak
    if is_moving:
        print("Robot sedang bergerak, tunggu dulu...")
        return

    # Abaikan klik di luar area grid
    if event.xdata is None or event.ydata is None:
        return

    # --- Konversi koordinat pixel ke koordinat grid ---
    col = int(event.xdata)              # Kolom = koordinat x
    row = GRID_ROWS - 1 - int(event.ydata)  # Baris = dibalik (y dari bawah ke atas)

    # Validasi batas grid
    if row < 0 or row >= GRID_ROWS or col < 0 or col >= GRID_COLS:
        return

    # Jangan klik obstacle
    if grid[row][col] == 1:
        print(f"Sel ({row},{col}) adalah obstacle, tidak bisa dijadikan goal!")
        return

    # Jangan klik posisi robot sendiri
    if [row, col] == robot_pos:
        print(f"Sel ({row},{col}) adalah posisi robot saat ini!")
        return

    # --- Set goal baru ---
    print(f"\n>>> Goal baru dipilih: ({row},{col})")
    goal_pos = [row, col]  # Buat list baru (karena sebelumnya bisa None)

    # --- Hitung ulang jalur A* ---
    calculate_new_path()

    # Gambar ulang tampilan
    draw_grid(ax)
    fig.canvas.draw()


# =============================================================================
# FUNGSI ANIMASI (ROBOT BERGERAK)
# =============================================================================

def animate(frame):
    global robot_pos, path_index, is_moving

    # Kalau tidak ada jalur, tidak perlu bergerak
    if not current_path or path_index >= len(current_path) - 1:
        is_moving = False
        draw_grid(ax)
        return

    # Robot sedang bergerak
    is_moving = True

    # Maju satu langkah di jalur
    path_index += 1
    next_pos = current_path[path_index]

    # Simpan posisi lama ke jejak
    robot_trail.append(tuple(robot_pos))

    # Update posisi robot
    robot_pos[0] = next_pos[0]
    robot_pos[1] = next_pos[1]

    # Cek apakah sudah sampai goal
    if goal_pos is not None and tuple(robot_pos) == tuple(goal_pos):
        is_moving = False
        print(f"\n✓ Robot sampai di goal {tuple(goal_pos)}!")
        print("Klik sel mana saja untuk set goal baru.")

    # Gambar ulang
    draw_grid(ax)


# PROGRAM UTAMA

if __name__ == "__main__":
    print("=" * 50)
    print("  ROBOT DELIVERY SIMULATION")
    print("=" * 50)
    print(f"Grid: {GRID_ROWS} x {GRID_COLS}")
    print(f"Robot mulai di: {tuple(robot_pos)}")
    print(f"Goal awal     : Belum dipilih (klik sel untuk menentukan)")
    print()
    print("Cara pakai:")
    print("  - Klik sembarang sel untuk menentukan goal pertama")
    print("  - Robot akan otomatis berjalan ke goal")
    print("  - Setelah sampai, klik sel lain untuk goal berikutnya")
    print("  - Sel gelap = obstacle (tidak bisa diklik)")
    print("=" * 50)

    # --- Buat figure Matplotlib ---
    fig, ax = plt.subplots(1, 1, figsize=(9, 7))
    fig.patch.set_facecolor('#EDF2F7')  # Warna background figure
    plt.subplots_adjust(right=0.78)      # Beri ruang untuk legend di kanan

    # --- Gambar grid awal (tanpa goal, tanpa jalur) ---
    draw_grid(ax)

    # --- Daftarkan handler klik mouse ---
    fig.canvas.mpl_connect('button_press_event', on_click)

    # --- Setup animasi ---
    # interval=400 berarti robot bergerak setiap 400 milidetik (0.4 detik)
    ani = animation.FuncAnimation(
        fig,
        animate,          # Fungsi yang dipanggil tiap frame
        interval=400,     # Kecepatan animasi (ms) — ubah lebih kecil = lebih cepat
        cache_frame_data=False
    )

    plt.tight_layout()
    plt.show()