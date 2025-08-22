import tkinter as tk
from tkinter import ttk, messagebox 
from mission_manager import MissionManager, MissionInsert

# TODO si stby supprimer la liste actuel de mission 
# TODO changer lamanière d'append une tache 

class MissionGUI:
    def __init__(self, window):
        self.window = window
        self.window.title("Mission GUI - Rover")

        print("cohoma_bridge test started")
        self.manager = MissionManager(verbose=True)
        self.manager.wait_ready()

        self.lat_offset = 43.563
        self.lon_offset = 1.484

        self.missions = []
        self.mission_id_counter = 1

        self.create_offset_ui()
        self.create_circle_ui()
        self.create_path_ui()
        self.create_buttons_and_list()

    # === OFFSET UI ===

    def create_offset_ui(self):
        self.offset_frame = tk.LabelFrame(self.window, text="Coordinate Offset (LTP origin)")
        self.offset_frame.grid(row=0, column=0, padx=10, pady=5, sticky="ew")

        tk.Label(self.offset_frame, text="Lat Offset:").grid(row=0, column=0)
        self.lat_offset_entry = tk.Entry(self.offset_frame, width=10)
        self.lat_offset_entry.insert(0, str(self.lat_offset))
        self.lat_offset_entry.grid(row=0, column=1)

        tk.Label(self.offset_frame, text="Lon Offset:").grid(row=0, column=2)
        self.lon_offset_entry = tk.Entry(self.offset_frame, width=10)
        self.lon_offset_entry.insert(0, str(self.lon_offset))
        self.lon_offset_entry.grid(row=0, column=3)

        update_button = tk.Button(self.offset_frame, text="Update Offset", command=self.update_offsets)
        update_button.grid(row=1, column=2, padx=10)

    def update_offsets(self):
        try:
            self.lat_offset = float(self.lat_offset_entry.get())
            self.lon_offset = float(self.lon_offset_entry.get())

            self.circle_lat_prefix.config(text=f"{self.lat_offset} +")
            self.circle_lon_prefix.config(text=f"{self.lon_offset} +")
            self.point_lat_prefix.config(text=f"{self.lat_offset} +")
            self.point_lon_prefix.config(text=f"{self.lon_offset} +")

            for i, (lat_entry, lon_entry) in enumerate(self.path_points):
                lat_label = self.path_point_frame.grid_slaves(row=i, column=1)[0]
                lon_label = self.path_point_frame.grid_slaves(row=i, column=3)[0]
                lat_label.config(text=f"{self.lat_offset} +")
                lon_label.config(text=f"{self.lon_offset} +")

        except ValueError:
            messagebox.showerror("Invalid Offset", "Please enter valid float values for offsets.")

    # === CIRCLE UI ===
    def create_circle_ui(self):
        self.circle_frame = tk.LabelFrame(self.window, text="Mission: Circle")
        self.circle_frame.grid(row=1, column=0, padx=10, pady=5, sticky="ew")
        
        self.circle_lat_prefix = tk.Label(self.circle_frame, text=f"{self.lat_offset} +")
        self.circle_lat_prefix.grid(row=0, column=0, padx=2, pady=1)
        self.circle_lat_entry = tk.Entry(self.circle_frame, width=10)
        self.circle_lat_entry.insert(0, "0.000387")
        self.circle_lat_entry.grid(row=0, column=1, padx=2, pady=1)

        self.circle_lon_prefix = tk.Label(self.circle_frame, text=f"{self.lon_offset} +")
        self.circle_lon_prefix.grid(row=0, column=2, padx=2, pady=1)
        self.circle_lon_entry = tk.Entry(self.circle_frame, width=10)
        self.circle_lon_entry.insert(0, "0.000481")
        self.circle_lon_entry.grid(row=0, column=3, padx=2, pady=1)

        self.circle_radius_label = tk.Label(self.circle_frame, text="Radius (m)")
        self.circle_radius_label.grid(row=1, column=0)
        self.circle_radius_entry = tk.Entry(self.circle_frame, width=10)
        self.circle_radius_entry.insert(0, "10")
        self.circle_radius_entry.grid(row=1, column=1)

        self.circle_duration_label = tk.Label(self.circle_frame, text="Duration (s)")
        self.circle_duration_label.grid(row=1, column=2)
        self.circle_duration_entry = tk.Entry(self.circle_frame, width=10)
        self.circle_duration_entry.insert(0, "35")
        self.circle_duration_entry.grid(row=1, column=3)

        self.circle_add_button = tk.Button(self.circle_frame, text="Add to Mission List", command=self.add_circle_mission)
        self.circle_add_button.grid(row=2, column=0, columnspan=4, pady=5)

    # === PATH OR POINT UI ===
    def create_path_ui(self):
        self.path_frame = tk.LabelFrame(self.window, text="Mission: Point/Path")
        self.path_frame.grid(row=2, column=0, padx=10, pady=5, sticky="ew")

        self.path_point_frame = tk.Frame(self.path_frame)
        self.path_point_frame.grid(row=0, column=0, columnspan=2)

        self.path_points = []
        self.add_path_point()
        self.add_path_point()

        self.add_point_button = tk.Button(self.path_frame, text="Add a point", command=self.add_path_point)
        self.add_point_button.grid(row=1, column=0, columnspan=2, pady=3)

        self.path_add_button = tk.Button(self.path_frame, text="Add to Mission List", command=self.add_path_or_point_mission)
        self.path_add_button.grid(row=2, column=0, columnspan=2, pady=5)

        self.remove_points_button = tk.Button(self.path_frame, text="Remove default points", command=self.remove_default_points)
        self.remove_points_button.grid(row=3, column=0, columnspan=2, pady=5)

    def add_path_point(self):
        row = len(self.path_points)

        label = tk.Label(self.path_point_frame, text=f"{row+1}.")
        label.grid(row=row, column=0, padx=2, pady=1)

        lat_prefix = tk.Label(self.path_point_frame, text=f"{self.lat_offset} +")
        lat_prefix.grid(row=row, column=1, padx=2, pady=1)
        lat_entry = tk.Entry(self.path_point_frame, width=10)
        lat_entry.insert(0, "0.000387")
        lat_entry.grid(row=row, column=2, padx=2, pady=1)

        lon_prefix = tk.Label(self.path_point_frame, text=f"{self.lon_offset} +")
        lon_prefix.grid(row=row, column=3, padx=2, pady=1)
        lon_entry = tk.Entry(self.path_point_frame, width=10)
        lon_entry.insert(0, "0.000481")
        lon_entry.grid(row=row, column=4, padx=2, pady=1)

        self.path_points.append((lat_entry, lon_entry))

    def add_path_or_point_mission(self):
        num_points = len(self.path_points)
        if num_points == 0:
            messagebox.showerror("Error", "Add at least one point before adding mission.")
            return
        elif num_points == 1:
            self.add_point_mission()
        else:
            self.add_path_mission()

    # === BUTTONS & LIST ===
    def create_buttons_and_list(self):
        self.list_frame = tk.LabelFrame(self.window, text="Mission List")
        self.list_frame.grid(row=3, column=0, pady=10, columnspan=2) 

        self.listbox = tk.Listbox(self.list_frame, width=80)
        self.listbox.grid(row=0, column=0, columnspan=2)

        self.buttons_frame = tk.Frame(self.list_frame)
        self.buttons_frame.grid(row=1, column=0, columnspan=2, pady=5)

        self.send_button = tk.Button(self.buttons_frame, text="Send All Missions", command=self.send_missions)
        self.send_button.grid(row=0, column=0, padx=5)

        self.delete_button = tk.Button(self.buttons_frame, text="Delete All", command=self.clear_all_missions)
        self.delete_button.grid(row=0, column=1, padx=5)
        
    def update_offsets(self):
        try:
            self.lat_offset = float(self.lat_offset_entry.get())
            self.lon_offset = float(self.lon_offset_entry.get())

            # Mettre à jour les labels dynamiques dans les autres frames
            self.circle_lat_prefix.config(text=f"{self.lat_offset} +")
            self.circle_lon_prefix.config(text=f"{self.lon_offset} +")

            for i, (lat_entry, lon_entry) in enumerate(self.path_points):
                lat_label = self.path_point_frame.grid_slaves(row=i, column=1)[0]
                lon_label = self.path_point_frame.grid_slaves(row=i, column=3)[0]
                lat_label.config(text=f"{self.lat_offset} +")
                lon_label.config(text=f"{self.lon_offset} +")

        except ValueError:
            messagebox.showerror("Invalid Offset", "Please enter valid float values for offsets.")

    # === MISSIONS ===
    def add_circle_mission(self):
        try:
            lat = round (self.lat_offset + float(self.circle_lat_entry.get()), 6)
            lon = round (self.lon_offset + float(self.circle_lon_entry.get()), 6)
            radius = float(self.circle_radius_entry.get())
            alt = self.manager.uav_data.alt
            duration = int(self.circle_duration_entry.get())

            mission = {
                'type': 'circle',
                'mission_id': self.mission_id_counter,
                "lat": lat,
                "lon": lon,
                "alt": alt,
                'radius': radius,
                'duration': duration
            }
            self.missions.append(mission)
            self.listbox.insert(tk.END, f"[CIRCLE] ID={mission['mission_id']} ({lat}, {lon}), r={radius}, alt={alt}, d={duration}s")
            self.mission_id_counter += 1
        except ValueError:
            messagebox.showerror("Error", "Invalid fields for circle mission.")

    def add_path_mission(self):
        path = []
        try:
            for lat_entry, lon_entry in self.path_points:
                lat = round (self.lat_offset + float(lat_entry.get()), 6)
                lon = round (self.lon_offset + float(lon_entry.get()), 6)
                path.append((lat, lon))

            alt = self.manager.uav_data.alt
            duration = -1

            mission = {
                'type': 'path',
                'mission_id': self.mission_id_counter,
                "path": path,
                "alt": alt,
                'duration': duration
            }
            self.missions.append(mission)

            point_indices = ", ".join(str(i + 1) for i in range(len(path)))
            self.listbox.insert(tk.END, f"[PATH] ID={mission['mission_id']} → {len(path)} points: ({point_indices}), d={duration}s")

            self.mission_id_counter += 1
        except ValueError:
            messagebox.showerror("Error", "Invalid fields for path mission.")

    def add_point_mission(self):
        path = []
        try:
            for lat_entry, lon_entry in self.path_points:
                lat = round (self.lat_offset + float(lat_entry.get()), 6)
                lon = round (self.lon_offset + float(lon_entry.get()), 6)

            alt = self.manager.uav_data.alt
            duration = -1

            mission = {
                'type': 'point',
                "lat": lat,
                "lon": lon,
                "alt": alt,
                'mission_id': self.mission_id_counter,
                'duration': duration
            }
            self.missions.append(mission)

            self.listbox.insert(tk.END, f"[POINT] ID={mission['mission_id']} ({lat}, {lon}), alt={alt}, d={duration}s")

            self.mission_id_counter += 1
        except ValueError:
            messagebox.showerror("Error", "Invalid fields for path mission.")

    def send_missions(self):
        try:
            for m in self.missions:
                if m["type"] == "circle":
                    print(f"AC_ID circle: {self.manager.ac_id}") 
                    self.manager.add_mission_circle(
                        mission_id=m["mission_id"],
                        lat=m["lat"],
                        lon=m["lon"],
                        alt=m["alt"],
                        radius=m["radius"],
                        duration=m["duration"]
                    )
                elif m["type"] == "point":
                    print(f"AC_ID point: {self.manager.ac_id}") 
                    self.manager.add_mission_point(
                        lat=m["lat"],
                        lon=m["lon"],
                        alt=m["alt"],
                        mission_id=m["mission_id"],
                        duration=m["duration"]
                    )
                elif m["type"] == "path":
                    print(f"AC_ID path: {self.manager.ac_id}") 
                    self.manager.add_mission_path(
                        mission_id=m["mission_id"],
                        path=m["path"],
                        alt=m["alt"],
                        duration=m["duration"]
                    )

            if self.manager.uav_data.FP_block != "Mission":
                self.manager.start_mission()
                print ("UAV: ", self.manager.uav_data.name)

            messagebox.showinfo("Success", "All the missions have been sent!")
        except Exception as e:
            messagebox.showerror("Sending error", str(e))
            manager.closing()

    def clear_all_missions(self):
        if messagebox.askyesno("Confirmation", "Delete all missions in Mission List?"):
            self.listbox.delete(0, tk.END)
            self.missions.clear()
    
    def remove_default_points(self):
        for widget in self.path_point_frame.winfo_children():
            widget.destroy()
        self.path_points.clear()

# === MAIN ===
if __name__ == "__main__":
    window = tk.Tk()
    app = MissionGUI(window)
    window.mainloop()