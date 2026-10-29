import tkinter as tk
from tkinter import ttk
import serial
import serial.tools.list_ports
import threading
import time
from datetime import datetime

class SonoGuardDashboard:
    def __init__(self, root):
        self.root = root
        self.root.title("SonoGuard - Real-Time Acoustic Hazard Monitor")
        self.root.geometry("850x620")
        self.root.configure(bg="#1E1E2E")

        self.serial_port = None
        self.is_running = False

        # Header Title
        title_label = tk.Label(
            root,
            text="SonoGuard Edge Acoustic Monitor",
            font=("Segoe UI", 18, "bold"),
            fg="#CDD6F4",
            bg="#1E1E2E"
        )
        title_label.pack(pady=12)

        # Connection Controls
        conn_frame = tk.Frame(root, bg="#1E1E2E")
        conn_frame.pack(fill="x", padx=20, pady=5)

        tk.Label(conn_frame, text="Serial Port:", font=("Segoe UI", 10), fg="#A6ADC8", bg="#1E1E2E").pack(side="left", padx=5)
        self.port_entry = tk.Entry(conn_frame, font=("Segoe UI", 10), width=10)
        self.port_entry.insert(0, "COM11")
        self.port_entry.pack(side="left", padx=5)

        self.btn_connect = tk.Button(
            conn_frame,
            text="Connect",
            font=("Segoe UI", 10, "bold"),
            bg="#A6E3A1",
            fg="#11111B",
            command=self.toggle_connection
        )
        self.btn_connect.pack(side="left", padx=10)

        self.status_label = tk.Label(conn_frame, text="Disconnected", font=("Segoe UI", 10), fg="#F38BA8", bg="#1E1E2E")
        self.status_label.pack(side="left", padx=10)

        # Telemetry Display Cards
        cards_frame = tk.Frame(root, bg="#1E1E2E")
        cards_frame.pack(fill="x", padx=20, pady=15)

        # Live Confidence Card
        card_ratio = tk.Frame(cards_frame, bg="#313244", relief="ridge", bd=1)
        card_ratio.pack(side="left", fill="both", expand=True, padx=5, ipady=10)
        tk.Label(card_ratio, text="Tonal Purity Ratio", font=("Segoe UI", 11), fg="#BAC2DE", bg="#313244").pack(pady=4)
        self.lbl_ratio = tk.Label(card_ratio, text="0.00", font=("Segoe UI", 26, "bold"), fg="#89B4FA", bg="#313244")
        self.lbl_ratio.pack()

        # Status Verification Card
        card_status = tk.Frame(cards_frame, bg="#313244", relief="ridge", bd=1)
        card_status.pack(side="left", fill="both", expand=True, padx=5, ipady=10)
        tk.Label(card_status, text="Hazard Verification", font=("Segoe UI", 11), fg="#BAC2DE", bg="#313244").pack(pady=4)
        self.lbl_status = tk.Label(card_status, text="MONITORING", font=("Segoe UI", 22, "bold"), fg="#A6E3A1", bg="#313244")
        self.lbl_status.pack()

        # Real-time Visual Level Meter
        meter_frame = tk.Frame(root, bg="#1E1E2E")
        meter_frame.pack(fill="x", padx=20, pady=10)
        tk.Label(meter_frame, text="Live Spectral Concentration:", font=("Segoe UI", 10), fg="#BAC2DE", bg="#1E1E2E").pack(anchor="w")
        
        self.progress_bar = ttk.Progressbar(meter_frame, orient="horizontal", length=800, mode="determinate")
        self.progress_bar.pack(fill="x", pady=6)

        # Event History Log
        log_frame = tk.Frame(root, bg="#1E1E2E")
        log_frame.pack(fill="both", expand=True, padx=20, pady=10)
        tk.Label(log_frame, text="Verified Alert History Log:", font=("Segoe UI", 11, "bold"), fg="#CDD6F4", bg="#1E1E2E").pack(anchor="w")

        columns = ("timestamp", "source", "ratio", "status")
        self.tree = ttk.Treeview(log_frame, columns=columns, show="headings", height=8)
        self.tree.heading("timestamp", text="Timestamp")
        self.tree.heading("source", text="Detected Frequency")
        self.tree.heading("ratio", text="Tonal Ratio")
        self.tree.heading("status", text="Verification Status")

        self.tree.column("timestamp", width=180, anchor="center")
        self.tree.column("source", width=180, anchor="center")
        self.tree.column("ratio", width=140, anchor="center")
        self.tree.column("status", width=220, anchor="center")
        self.tree.pack(side="left", fill="both", expand=True)

        scrollbar = ttk.Scrollbar(log_frame, orient="vertical", command=self.tree.yview)
        self.tree.configure(yscroll=scrollbar.set)
        scrollbar.pack(side="right", fill="y")

    def toggle_connection(self):
        if not self.is_running:
            port = self.port_entry.get().strip()
            try:
                self.serial_port = serial.Serial(port, 115200, timeout=1)
                self.is_running = True
                self.btn_connect.config(text="Disconnect", bg="#F38BA8")
                self.status_label.config(text="Connected to " + port, fg="#A6E3A1")
                self.read_thread = threading.Thread(target=self.read_serial_data, daemon=True)
                self.read_thread.start()
            except Exception as e:
                self.status_label.config(text="Failed to connect", fg="#F38BA8")
        else:
            self.is_running = False
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.close()
            self.btn_connect.config(text="Connect", bg="#A6E3A1")
            self.status_label.config(text="Disconnected", fg="#F38BA8")

    def read_serial_data(self):
        last_log_time = 0
        while self.is_running:
            if self.serial_port and self.serial_port.in_waiting:
                try:
                    line = self.serial_port.readline().decode("utf-8", errors="ignore").strip()
                    if not line:
                        continue

                    # Parse output lines containing Ratio
                    if "Ratio:" in line:
                        parts = line.split("Ratio:")
                        if len(parts) > 1:
                            val_str = parts[1].strip()
                            ratio_val = float(val_str)
                            self.root.after(0, self.update_telemetry, ratio_val)

                            # Log confirmed alert when ratio is high and throttled by time
                            current_time = time.time()
                            if ratio_val >= 0.85 and (current_time - last_log_time > 2.0):
                                last_log_time = current_time
                                self.root.after(0, self.add_log_entry, "1250 Hz Tone", ratio_val, "HAZARD CONFIRMED")
                except Exception:
                    pass
            time.sleep(0.01)

    def update_telemetry(self, ratio_val):
        self.lbl_ratio.config(text=f"{ratio_val:.2f}")
        percent = min(100, int(ratio_val * 100))
        self.progress_bar["value"] = percent

        if ratio_val >= 0.80:
            self.lbl_status.config(text="HAZARD DETECTED", fg="#F38BA8")
            self.lbl_ratio.config(fg="#F38BA8")
        else:
            self.lbl_status.config(text="CLEAR / SAFE", fg="#A6E3A1")
            self.lbl_ratio.config(fg="#89B4FA")

    def add_log_entry(self, source, ratio_val, status):
        now_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self.tree.insert("", 0, values=(now_str, source, f"{ratio_val:.2f}", status))

if __name__ == "__main__":
    app_root = tk.Tk()
    app = SonoGuardDashboard(app_root)
    app_root.mainloop()