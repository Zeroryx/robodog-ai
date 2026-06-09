# File ini berisi implementasi algoritma A* (A-Star) untuk pathfinding.
#
# Cara kerja A*:
#   1. Mulai dari posisi START
#   2. Eksplorasi sel-sel tetangga (atas, bawah, kiri, kanan)
#   3. Setiap sel diberi nilai f(n) = g(n) + h(n):
#        - g(n) = biaya dari START ke sel saat ini
#        - h(n) = estimasi biaya dari sel saat ini ke GOAL (heuristik)
#   4. Selalu pilih sel dengan nilai f terkecil
#   5. Berhenti ketika mencapai GOAL
#   6. Rekonstruksi jalur dari GOAL ke START
#
# Heuristik yang digunakan: Manhattan Distance
#   h = |baris_sekarang - baris_goal| + |kolom_sekarang - kolom_goal|

import heapq  # Modul untuk priority queue (min-heap)
from navigation.gridmap import is_walkable


def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def astar(grid, start, goal):
    # --- Validasi awal ---
    # Pastikan start dan goal bukan obstacle
    if not is_walkable(grid, start[0], start[1]):
        print("ERROR: Posisi start adalah obstacle!")
        return []
    if not is_walkable(grid, goal[0], goal[1]):
        print("ERROR: Posisi goal adalah obstacle!")
        return []

    # --- Priority Queue (Open Set) ---
    # Format heap: (f_value, (baris, kolom))
    # heapq selalu mengeluarkan elemen dengan nilai terkecil lebih dulu
    open_set = []
    heapq.heappush(open_set, (0, start))  # Masukkan posisi start

    # --- Tracking dari mana kita datang ---
    # came_from[sel] = sel sebelumnya dalam jalur terbaik
    came_from = {}

    # --- Biaya dari start ke setiap sel ---
    # g_cost[sel] = total biaya gerakan dari start
    g_cost = {}
    g_cost[start] = 0

    # --- Set sel yang sudah dieksplorasi ---
    visited = set()

    # --- 4 Arah Gerakan: Atas, Bawah, Kiri, Kanan ---
    directions = [
        (-1,  0),  # Atas
        ( 1,  0),  # Bawah
        ( 0, -1),  # Kiri
        ( 0,  1),  # Kanan
    ]

    # --- Loop Utama A* ---
    while open_set:
        # Ambil sel dengan f_value terkecil dari priority queue
        current_f, current = heapq.heappop(open_set)

        # Jika sudah sampai goal, rekonstruksi jalur
        if current == goal:
            return reconstruct_path(came_from, current)

        # Tandai sel ini sudah dikunjungi
        if current in visited:
            continue  # Skip jika sudah pernah diproses
        visited.add(current)

        # --- Eksplorasi 4 Tetangga ---
        for dr, dc in directions:
            neighbor = (current[0] + dr, current[1] + dc)

            # Skip jika tetangga tidak bisa dilewati
            if not is_walkable(grid, neighbor[0], neighbor[1]):
                continue

            # Skip jika tetangga sudah dikunjungi
            if neighbor in visited:
                continue

            # Hitung g_cost baru ke tetangga ini
            # Setiap langkah memiliki biaya 1
            new_g = g_cost[current] + 1

            # Apakah rute ini lebih baik dari yang sudah ada?
            if neighbor not in g_cost or new_g < g_cost[neighbor]:
                # Perbarui informasi tetangga
                g_cost[neighbor] = new_g
                came_from[neighbor] = current  # Catat dari mana kita datang

                # Hitung f = g + h
                f = new_g + heuristic(neighbor, goal)

                # Masukkan ke priority queue
                heapq.heappush(open_set, (f, neighbor))

    # Jika loop selesai tanpa menemukan goal
    print("Tidak ada jalur ditemukan!")
    return []


def reconstruct_path(came_from, current):
    path = []

    # Ikuti jejak came_from dari goal ke start
    while current in came_from:
        path.append(current)
        current = came_from[current]

    path.append(current)  # Tambahkan posisi start
    path.reverse()        # Balik urutan: dari start ke goal

    return path