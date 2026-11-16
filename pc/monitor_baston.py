import csv
import math
import queue
import socket
import threading
import time
from collections import deque
from datetime import datetime
from pathlib import Path
import tkinter as tk
from tkinter import ttk, messagebox

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

UDP_PORT = 4210
BUFFER_SECONDS = 15
EXPECTED_FIELDS = 20

CSV_FIELDS = [
    "pc_time_iso",
    "sequence",
    "time_ms",
    "fsr_raw",
    "contact",
    "ax_ms2",
    "ay_ms2",
    "az_ms2",
    "gx_deg_s",
    "gy_deg_s",
    "gz_deg_s",
    "cane_cadence_cpm",
    "contact_time_ms",
    "swing_time_ms",
    "cycle_time_ms",
    "cycle_cv_percent",
    "base_spm",
    "target_spm",
    "percentage",
    "device_on",
    "cue_active",
]

NUMERIC_FIELDS = {
    "sequence": int,
    "time_ms": int,
    "fsr_raw": int,
    "contact": int,
    "ax_ms2": float,
    "ay_ms2": float,
    "az_ms2": float,
    "gx_deg_s": float,
    "gy_deg_s": float,
    "gz_deg_s": float,
    "cane_cadence_cpm": float,
    "contact_time_ms": int,
    "swing_time_ms": int,
    "cycle_time_ms": int,
    "cycle_cv_percent": float,
    "base_spm": int,
    "target_spm": int,
    "percentage": int,
    "device_on": int,
    "cue_active": int,
}


def safe_float(text):
    try:
        return float(text)
    except Exception:
        return float("nan")


def parse_packet(text):
    parts = [p.strip() for p in text.split(",")]
    if len(parts) != EXPECTED_FIELDS:
        raise ValueError(f"Se esperaban {EXPECTED_FIELDS} campos y llegaron {len(parts)}")

    keys = CSV_FIELDS[1:]
    row = {}
    for key, value in zip(keys, parts):
        converter = NUMERIC_FIELDS[key]
        if converter is int:
            row[key] = int(float(value))
        else:
            row[key] = safe_float(value)

    row["pc_time_iso"] = datetime.now().isoformat(timespec="milliseconds")
    return row


class ReceiverThread(threading.Thread):
    def __init__(self, data_queue, status_queue, stop_event):
        super().__init__(daemon=True)
        self.data_queue = data_queue
        self.status_queue = status_queue
        self.stop_event = stop_event

    def run(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.bind(("0.0.0.0", UDP_PORT))
        except OSError as e:
            self.status_queue.put(("error", f"No se pudo abrir UDP {UDP_PORT}: {e}"))
            return

        sock.settimeout(0.5)
        self.status_queue.put(("listening", f"Escuchando UDP en puerto {UDP_PORT}"))

        while not self.stop_event.is_set():
            try:
                data, addr = sock.recvfrom(2048)
                text = data.decode("utf-8", errors="replace").strip()
                row = parse_packet(text)
                self.data_queue.put((row, addr))
            except socket.timeout:
                continue
            except ValueError as e:
                self.status_queue.put(("packet_error", str(e)))
            except OSError:
                break
            except Exception as e:
                self.status_queue.put(("packet_error", f"Error de recepción: {e}"))

        sock.close()


class BastonMonitor(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Monitor del Bastón de Parkinson")
        self.geometry("1450x900")
        self.minsize(1100, 720)

        self.data_queue = queue.Queue()
        self.status_queue = queue.Queue()
        self.stop_event = threading.Event()

        self.recording = False
        self.csv_file = None
        self.csv_writer = None
        self.csv_path = None

        self.last_packet_monotonic = None
        self.last_addr = None
        self.last_sequence = None
        self.received_count = 0
        self.lost_count = 0

        self.rate_times = deque(maxlen=500)

        max_points = BUFFER_SECONDS * 120
        self.t_data = deque(maxlen=max_points)
        self.fsr_data = deque(maxlen=max_points)
        self.contact_data = deque(maxlen=max_points)

        self.ax_data = deque(maxlen=max_points)
        self.ay_data = deque(maxlen=max_points)
        self.az_data = deque(maxlen=max_points)

        self.gx_data = deque(maxlen=max_points)
        self.gy_data = deque(maxlen=max_points)
        self.gz_data = deque(maxlen=max_points)

        self.cadence_data = deque(maxlen=max_points)
        self.cv_data = deque(maxlen=max_points)

        self.first_device_time_ms = None
        self.latest = None

        self._build_ui()

        self.receiver = ReceiverThread(
            self.data_queue,
            self.status_queue,
            self.stop_event,
        )
        self.receiver.start()

        self.protocol("WM_DELETE_WINDOW", self.on_close)
        self.after(50, self.process_queues)
        self.after(200, self.refresh_plots)
        self.after(500, self.refresh_status)

    def _build_ui(self):
        top = ttk.Frame(self, padding=8)
        top.pack(fill="x")

        self.connection_var = tk.StringVar(value="Esperando datos del ESP8266…")
        ttk.Label(top, textvariable=self.connection_var, font=("Segoe UI", 11, "bold")).pack(side="left")

        ttk.Button(top, text="Nueva grabación CSV", command=self.start_recording).pack(side="right", padx=(6, 0))
        ttk.Button(top, text="Detener grabación", command=self.stop_recording).pack(side="right")

        self.record_var = tk.StringVar(value="CSV: detenido")
        ttk.Label(top, textvariable=self.record_var).pack(side="right", padx=12)

        stats = ttk.LabelFrame(self, text="Valores en tiempo real", padding=8)
        stats.pack(fill="x", padx=8, pady=(0, 8))

        self.value_vars = {}
        items = [
            ("FSR", "fsr"),
            ("Contacto", "contact"),
            ("Ax", "ax"),
            ("Ay", "ay"),
            ("Az", "az"),
            ("Gx", "gx"),
            ("Gy", "gy"),
            ("Gz", "gz"),
            ("Cadencia bastón", "cadence"),
            ("Contacto ms", "contact_ms"),
            ("Balanceo ms", "swing_ms"),
            ("Ciclo ms", "cycle_ms"),
            ("CV ciclo", "cv"),
            ("Base SPM", "base"),
            ("Objetivo SPM", "target"),
            ("Modo", "percentage"),
            ("Cueing", "device"),
            ("Pulso cue", "cue"),
            ("Hz recibidos", "hz"),
            ("Paquetes perdidos", "lost"),
        ]

        columns = 5
        for i, (label, key) in enumerate(items):
            r = i // columns
            c = i % columns

            cell = ttk.Frame(stats, padding=(5, 3))
            cell.grid(row=r, column=c, sticky="ew")
            stats.columnconfigure(c, weight=1)

            ttk.Label(cell, text=label, font=("Segoe UI", 8)).pack(anchor="w")
            var = tk.StringVar(value="—")
            ttk.Label(cell, textvariable=var, font=("Segoe UI", 11, "bold")).pack(anchor="w")
            self.value_vars[key] = var

        graphs = ttk.Frame(self, padding=(8, 0, 8, 8))
        graphs.pack(fill="both", expand=True)

        for r in range(2):
            graphs.rowconfigure(r, weight=1)
        for c in range(2):
            graphs.columnconfigure(c, weight=1)

        self.fig_fsr, self.ax_fsr, self.canvas_fsr = self.make_plot(graphs, "FSR y contacto", 0, 0)
        self.fig_acc, self.ax_acc, self.canvas_acc = self.make_plot(graphs, "Aceleración", 0, 1)
        self.fig_gyro, self.ax_gyro, self.canvas_gyro = self.make_plot(graphs, "Velocidad angular", 1, 0)
        self.fig_cad, self.ax_cad, self.canvas_cad = self.make_plot(graphs, "Cadencia del bastón y CV", 1, 1)

        self.ax_fsr.set_ylabel("FSR")
        self.ax_acc.set_ylabel("m/s²")
        self.ax_gyro.set_ylabel("°/s")
        self.ax_cad.set_ylabel("CPM / %")

        for ax in (self.ax_fsr, self.ax_acc, self.ax_gyro, self.ax_cad):
            ax.set_xlabel("Tiempo (s)")
            ax.grid(True, alpha=0.25)

    def make_plot(self, parent, title, row, col):
        frame = ttk.LabelFrame(parent, text=title, padding=4)
        frame.grid(row=row, column=col, sticky="nsew", padx=4, pady=4)

        fig = Figure(figsize=(5, 3), dpi=100)
        ax = fig.add_subplot(111)
        fig.tight_layout(pad=2.0)

        canvas = FigureCanvasTkAgg(fig, master=frame)
        canvas.get_tk_widget().pack(fill="both", expand=True)

        return fig, ax, canvas

    def start_recording(self):
        self.stop_recording()

        folder = Path.cwd() / "registros"
        folder.mkdir(parents=True, exist_ok=True)

        name = datetime.now().strftime("baston_%Y%m%d_%H%M%S.csv")
        self.csv_path = folder / name
        self.csv_file = open(self.csv_path, "w", newline="", encoding="utf-8-sig")
        self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=CSV_FIELDS)
        self.csv_writer.writeheader()
        self.csv_file.flush()

        self.recording = True
        self.record_var.set(f"CSV: grabando → {self.csv_path.name}")

    def stop_recording(self):
        if self.csv_file is not None:
            try:
                self.csv_file.flush()
                self.csv_file.close()
            except Exception:
                pass

        was_path = self.csv_path
        self.csv_file = None
        self.csv_writer = None
        self.recording = False

        if was_path:
            self.record_var.set(f"CSV guardado: {was_path.name}")
        else:
            self.record_var.set("CSV: detenido")

    def process_queues(self):
        try:
            while True:
                kind, message = self.status_queue.get_nowait()
                if kind == "error":
                    messagebox.showerror("UDP", message)
                    self.connection_var.set(message)
                elif kind == "listening":
                    self.connection_var.set(message + " — conecta la laptop a Baston_Parkinson")
        except queue.Empty:
            pass

        processed = 0
        try:
            while processed < 500:
                row, addr = self.data_queue.get_nowait()
                self.consume_row(row, addr)
                processed += 1
        except queue.Empty:
            pass

        self.after(50, self.process_queues)

    def consume_row(self, row, addr):
        now = time.monotonic()
        self.last_packet_monotonic = now
        self.last_addr = addr
        self.latest = row

        self.received_count += 1
        self.rate_times.append(now)

        seq = row["sequence"]
        if self.last_sequence is not None and seq > self.last_sequence + 1:
            self.lost_count += seq - self.last_sequence - 1
        self.last_sequence = seq

        if self.first_device_time_ms is None:
            self.first_device_time_ms = row["time_ms"]

        t = (row["time_ms"] - self.first_device_time_ms) / 1000.0

        self.t_data.append(t)
        self.fsr_data.append(row["fsr_raw"])
        self.contact_data.append(row["contact"])

        self.ax_data.append(row["ax_ms2"])
        self.ay_data.append(row["ay_ms2"])
        self.az_data.append(row["az_ms2"])

        self.gx_data.append(row["gx_deg_s"])
        self.gy_data.append(row["gy_deg_s"])
        self.gz_data.append(row["gz_deg_s"])

        self.cadence_data.append(row["cane_cadence_cpm"])
        self.cv_data.append(row["cycle_cv_percent"])

        if self.recording and self.csv_writer:
            try:
                self.csv_writer.writerow({field: row.get(field, "") for field in CSV_FIELDS})
                if self.received_count % 25 == 0:
                    self.csv_file.flush()
            except Exception as e:
                self.stop_recording()
                messagebox.showerror("CSV", f"Se detuvo la grabación: {e}")

        self.update_numeric_values(row)

    def update_numeric_values(self, r):
        self.value_vars["fsr"].set(str(r["fsr_raw"]))
        self.value_vars["contact"].set("SÍ" if r["contact"] else "NO")
        self.value_vars["ax"].set(f'{r["ax_ms2"]:.2f} m/s²')
        self.value_vars["ay"].set(f'{r["ay_ms2"]:.2f} m/s²')
        self.value_vars["az"].set(f'{r["az_ms2"]:.2f} m/s²')
        self.value_vars["gx"].set(f'{r["gx_deg_s"]:.2f} °/s')
        self.value_vars["gy"].set(f'{r["gy_deg_s"]:.2f} °/s')
        self.value_vars["gz"].set(f'{r["gz_deg_s"]:.2f} °/s')
        self.value_vars["cadence"].set(f'{r["cane_cadence_cpm"]:.1f} CPM')
        self.value_vars["contact_ms"].set(f'{r["contact_time_ms"]} ms')
        self.value_vars["swing_ms"].set(f'{r["swing_time_ms"]} ms')
        self.value_vars["cycle_ms"].set(f'{r["cycle_time_ms"]} ms')
        self.value_vars["cv"].set(f'{r["cycle_cv_percent"]:.1f} %')
        self.value_vars["base"].set(str(r["base_spm"]))
        self.value_vars["target"].set(str(r["target_spm"]))
        self.value_vars["percentage"].set(f'{r["percentage"]}%')
        self.value_vars["device"].set("ON" if r["device_on"] else "OFF")
        self.value_vars["cue"].set("ACTIVO" if r["cue_active"] else "—")
        self.value_vars["lost"].set(str(self.lost_count))

    def refresh_status(self):
        now = time.monotonic()

        while self.rate_times and now - self.rate_times[0] > 2.0:
            self.rate_times.popleft()

        if len(self.rate_times) >= 2:
            dt = self.rate_times[-1] - self.rate_times[0]
            hz = (len(self.rate_times) - 1) / dt if dt > 0 else 0
        else:
            hz = 0

        self.value_vars["hz"].set(f"{hz:.1f} Hz")

        if self.last_packet_monotonic is None:
            self.connection_var.set(f"Esperando datos UDP en puerto {UDP_PORT}…")
        else:
            age = now - self.last_packet_monotonic
            if age < 1.5:
                ip = self.last_addr[0] if self.last_addr else "ESP8266"
                self.connection_var.set(f"RECIBIENDO DATOS — {ip} — {hz:.1f} Hz")
            else:
                self.connection_var.set(f"SIN DATOS desde hace {age:.1f} s")

        self.after(500, self.refresh_status)

    @staticmethod
    def finite_series(values):
        return [float(v) if math.isfinite(float(v)) else float("nan") for v in values]

    def refresh_plots(self):
        if len(self.t_data) >= 2:
            t = list(self.t_data)

            # FSR
            self.ax_fsr.clear()
            self.ax_fsr.plot(t, list(self.fsr_data), label="FSR")
            self.ax_fsr.fill_between(
                t,
                0,
                max(max(self.fsr_data), 1),
                where=[bool(x) for x in self.contact_data],
                alpha=0.12,
                label="Contacto",
            )
            self.ax_fsr.set_title("FSR / contacto")
            self.ax_fsr.set_ylabel("FSR")
            self.ax_fsr.set_xlabel("Tiempo (s)")
            self.ax_fsr.grid(True, alpha=0.25)
            self.ax_fsr.legend(loc="upper right", fontsize=8)

            # Aceleración
            self.ax_acc.clear()
            self.ax_acc.plot(t, self.finite_series(self.ax_data), label="Ax")
            self.ax_acc.plot(t, self.finite_series(self.ay_data), label="Ay")
            self.ax_acc.plot(t, self.finite_series(self.az_data), label="Az")
            self.ax_acc.set_title("Aceleración")
            self.ax_acc.set_ylabel("m/s²")
            self.ax_acc.set_xlabel("Tiempo (s)")
            self.ax_acc.grid(True, alpha=0.25)
            self.ax_acc.legend(loc="upper right", fontsize=8, ncol=3)

            # Giroscopio
            self.ax_gyro.clear()
            self.ax_gyro.plot(t, self.finite_series(self.gx_data), label="Gx")
            self.ax_gyro.plot(t, self.finite_series(self.gy_data), label="Gy")
            self.ax_gyro.plot(t, self.finite_series(self.gz_data), label="Gz")
            self.ax_gyro.set_title("Velocidad angular")
            self.ax_gyro.set_ylabel("°/s")
            self.ax_gyro.set_xlabel("Tiempo (s)")
            self.ax_gyro.grid(True, alpha=0.25)
            self.ax_gyro.legend(loc="upper right", fontsize=8, ncol=3)

            # Cadencia/CV
            self.ax_cad.clear()
            self.ax_cad.plot(t, self.finite_series(self.cadence_data), label="Cadencia CPM")
            self.ax_cad.plot(t, self.finite_series(self.cv_data), label="CV %")
            self.ax_cad.set_title("Cadencia del bastón / variabilidad")
            self.ax_cad.set_ylabel("CPM / %")
            self.ax_cad.set_xlabel("Tiempo (s)")
            self.ax_cad.grid(True, alpha=0.25)
            self.ax_cad.legend(loc="upper right", fontsize=8)

            self.fig_fsr.tight_layout(pad=1.5)
            self.fig_acc.tight_layout(pad=1.5)
            self.fig_gyro.tight_layout(pad=1.5)
            self.fig_cad.tight_layout(pad=1.5)

            self.canvas_fsr.draw_idle()
            self.canvas_acc.draw_idle()
            self.canvas_gyro.draw_idle()
            self.canvas_cad.draw_idle()

        self.after(200, self.refresh_plots)

    def on_close(self):
        self.stop_event.set()
        self.stop_recording()
        self.destroy()


if __name__ == "__main__":
    app = BastonMonitor()
    app.mainloop()
