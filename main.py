"""
sensor_tester.py
Single-file Sensor Tester GUI (cross-platform, simulates hardware when native libs missing).

Features:
- Scan for devices (Serial ports + simulated I2C/SPI)
- Select device, pin, sensor type
- Run sampling tests (live plotting)
- Compute min/max/mean/median/std, empirical resolution, observed sample rate
- Export CSV
- Simulation mode if native hardware libraries not available

Usage:
    python sensor_tester.py
"""

import threading
import time
import csv
import math
import statistics
from datetime import datetime
from collections import deque
import sys
import os

# UI libs
try:
    import customtkinter as ctk
except Exception as e:
    raise SystemExit("Please install customtkinter: pip install customtkinter")

# Optional hardware libs (may not exist on Windows)
try:
    import serial.tools.list_ports as list_ports
    import serial
except Exception:
    serial = None
    list_ports = None

try:
    from smbus2 import SMBus
except Exception:
    SMBus = None

try:
    import spidev
except Exception:
    spidev = None

# plotting
try:
    import matplotlib
    matplotlib.use("TkAgg")
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    import matplotlib.pyplot as plt
except Exception:
    plt = None

import numpy as np

# ---------- Simulators / Safe wrappers ----------

class I2CSim:
    """Simple I2C simulator: pretend there's an MPU6050 at 0x68 and BMP280 at 0x76"""
    def __init__(self, busnum=0):
        self.busnum = busnum
        # Simulated devices
        self.devices = {
            0x68: "MPU6050 (sim)",
            0x76: "BME280 (sim)"
        }

    def scan(self):
        return list(self.devices.keys())

    def read_accel(self, addr):
        # return tuple ax,ay,az in g
        # generate low-noise random walk + gravity on z
        t = time.time()
        ax = 0.02 * math.sin(t*2.1) + np.random.normal(0, 0.005)
        ay = 0.01 * math.sin(t*1.7) + np.random.normal(0, 0.005)
        az = 1.0 + 0.02 * math.sin(t*0.9) + np.random.normal(0, 0.006)
        return ax, ay, az

    def read_gyro(self, addr):
        # return gx, gy, gz in deg/s
        t = time.time()
        gx = 1.0 * math.sin(t*2.5) + np.random.normal(0, 0.1)
        gy = 0.5 * math.sin(t*1.9) + np.random.normal(0, 0.1)
        gz = 0.2 * math.sin(t*1.3) + np.random.normal(0, 0.05)
        return gx, gy, gz

    def read_adc(self, addr):
        # return a simulated ADC reading (signed 16-bit style)
        # Sine wave + noise
        v = 1000.0 * (0.5 + 0.5*math.sin(time.time()*2.0)) + np.random.normal(0, 5)
        return int(v)

    def read_temp_pres(self, addr):
        # return temp (C) and pressure (Pa) raw-like values
        temp = 24.0 + 0.6*math.sin(time.time()*0.6) + np.random.normal(0, 0.05)
        pres = 101325 + 15*math.sin(time.time()*0.2) + np.random.normal(0, 2)
        return temp, pres

class SpiSim:
    def open(self, bus, device):
        self.opened = True
    def xfer2(self, data):
        # return fake bytes (e.g. ADC)
        # simulate variable reading
        val = int(2048 + 1024*math.sin(time.time()*3.1) + np.random.normal(0, 20))
        # return 2 bytes big endian
        hi = (val >> 8) & 0xFF
        lo = val & 0xFF
        return [hi, lo]
    def close(self):
        self.opened = False

# Serial simulator - pretends to be a serial dev that emits numbers when prompted
class SerialSimPort:
    def __init__(self, name="/dev/sim0"):
        self.name = name
        self.baudrate = 115200
        self._counter = 0
        self._last_write = b""

    def write(self, b):
        # remember last write to allow different simulated responses
        try:
            self._last_write = b
        except Exception:
            self._last_write = b
        pass

    def readline(self):
        # produce a line with a float
        self._counter += 1
        # if a specific command was written recently, vary output
        cmd = None
        try:
            cmd = self._last_write.decode("utf-8").strip().lower()
        except Exception:
            cmd = None
        t = time.time()
        if cmd and ("dist" in cmd or "ultra" in cmd or "hc" in cmd):
            # simulate ultrasonic distance in cm (20..400)
            val = 120.0 + 50.0 * math.sin(t*1.1) + np.random.normal(0, 3.0)
        elif cmd and ("ir" in cmd):
            # IR digital sensor: mostly 0 or 1
            val = float(np.random.choice([0,1], p=[0.7,0.3]))
        elif cmd and ("gyro" in cmd):
            # gyro degrees/sec
            val = 5.0 * math.sin(t*2.3) + np.random.normal(0, 0.5)
        else:
            # default analog-like signal
            val = 2.5*math.sin(t*1.3) + np.random.normal(0, 0.05)
        return f"{val:.4f}\n".encode("utf-8")

    def close(self):
        pass

# Choose which implementations to use
I2C_AVAILABLE = SMBus is not None
SPI_AVAILABLE = spidev is not None
SERIAL_AVAILABLE = serial is not None and list_ports is not None

# ---------- Sensor service - detection and reading ----------

class SensorService:
    def __init__(self):
        # use real SMBus if available, else simulator
        self.i2c_impl = None
        if I2C_AVAILABLE:
            try:
                # we will open bus 1 by default if exists
                self.i2c_impl = SMBus(1)
            except Exception:
                self.i2c_impl = I2CSim()
        else:
            self.i2c_impl = I2CSim()

        # SPI
        self.spi_impl = None
        if SPI_AVAILABLE:
            try:
                s = spidev.SpiDev()
                # don't open yet; will open on demand
                self.spi_impl = s
            except Exception:
                self.spi_impl = SpiSim()
        else:
            self.spi_impl = SpiSim()

        # serial listing
        self.serial_impl = SERIAL_AVAILABLE

    def detect_devices(self):
        devices = []
        # Serial ports
        if self.serial_impl:
            ports = list_ports.comports()
            for p in ports:
                devices.append({
                    "bus": "serial",
                    "id": p.device,
                    "description": p.description or f"Serial {p.device}"
                })
        else:
            # simulate a serial device occasionally
            devices.append({"bus":"serial", "id":"SIM_SERIAL_0", "description":"Simulated Serial Device"})

        # I2C devices - simulated scan
        if I2C_AVAILABLE and isinstance(self.i2c_impl, SMBus):
            # Basic naive scan: try addresses 0x03..0x77
            found = []
            for addr in range(0x03, 0x78):
                try:
                    # quick read which may raise
                    self.i2c_impl.read_byte(addr)
                    found.append(addr)
                except Exception:
                    pass
            if found:
                devices.append({"bus":"i2c", "id": f"i2c-1", "addresses": found})
        else:
            # simulator: return simulated addresses
            sim = getattr(self.i2c_impl, "devices", {0x68:"MPU6050 (sim)", 0x76:"BME280 (sim)"})
            devices.append({"bus":"i2c", "id":"i2c-sim", "addresses": list(sim.keys())})

        # SPI devices (we don't enumerate /dev spidev on Windows) - add a single simulated device
        if SPI_AVAILABLE:
            devices.append({"bus":"spi", "id":"spi-0.0"})
        else:
            devices.append({"bus":"spi", "id":"spi-sim-0.0"})

        return devices

    def read_once(self, device_spec, sensor_type, pin=None):
        """
        device_spec: dict as returned by detect_devices entries
        sensor_type: 'auto','accelerometer','pressure','adc','temp','generic','serial'
        pin: optional pin selection
        Returns a dict with numeric reading(s)
        """
        bus = device_spec.get("bus")
        # SERIAL
        if bus == "serial":
            # support a few higher-level sensor commands for serial-attached devices
            desired = (sensor_type or "").lower()
            # choose a short ASCII command to send to real hardware (best-effort)
            cmd_base = "READ"
            if desired in ("ultrasonic","hc-sr04","hc05","distance"):
                cmd_base = "DIST"
            elif desired in ("ir","ir_sensor"):
                cmd_base = "IR"
            elif desired in ("gyro",):
                cmd_base = "GYRO"

            # include pin info if provided (e.g. "A0", "D2", "TRIG:ECHO")
            pin_arg = (pin or "").strip()
            if pin_arg and pin_arg.upper() != "AUTO":
                cmd = (cmd_base + " " + pin_arg + "\n").encode()
            else:
                cmd = (cmd_base + "\n").encode()

            if SERIAL_AVAILABLE:
                # try to open a real serial port and ask for a reading
                try:
                    ser = serial.Serial(device_spec["id"], 115200, timeout=0.5)
                    try:
                        ser.write(cmd)
                    except Exception:
                        # some devices ignore write
                        pass
                    line = ser.readline().decode("utf-8").strip()
                    ser.close()
                    try:
                        return {"value": float(line)}
                    except Exception:
                        return {"raw": line}
                except Exception:
                    # fallback to simulator below
                    pass

            # Simulator path (or fallback)
            sim = SerialSimPort(device_spec.get("id","/dev/sim0"))
            # write the same command to the simulator so it can tailor its response
            try:
                sim.write(cmd)
            except Exception:
                pass
            line = sim.readline().decode().strip()
            try:
                return {"value": float(line)}
            except Exception:
                return {"raw": line}

        # I2C
        if bus == "i2c":
            # choose address if present
            addrs = device_spec.get("addresses", [])
            addr = addrs[0] if addrs else (list(getattr(self.i2c_impl, "devices", {}).keys())[0])
            # auto detect: prefer accel if MPU found at 0x68
            if sensor_type in ("accelerometer","auto"):
                if addr in (0x68, 0x69) or ("MPU" in str(self.i2c_impl.__class__)):
                    # try to use real read if using SMBus
                    if I2C_AVAILABLE and isinstance(self.i2c_impl, SMBus):
                        try:
                            # read raw accel regs (MPU6050) 0x3B..0x40
                            data = self.i2c_impl.read_i2c_block_data(addr, 0x3B, 6)
                            ax = int.from_bytes(bytes(data[0:2]), byteorder='big', signed=True)/16384.0
                            ay = int.from_bytes(bytes(data[2:4]), byteorder='big', signed=True)/16384.0
                            az = int.from_bytes(bytes(data[4:6]), byteorder='big', signed=True)/16384.0
                            return {"ax":ax,"ay":ay,"az":az}
                        except Exception:
                            # fallback to sim
                            ax,ay,az = self.i2c_impl.read_accel(addr)
                            return {"ax":ax,"ay":ay,"az":az}
                    else:
                        ax,ay,az = self.i2c_impl.read_accel(addr)
                        return {"ax":ax,"ay":ay,"az":az}

                # Gyro (MPU family)
                if sensor_type in ("gyro","auto"):
                    if addr in (0x68, 0x69) or ("MPU" in str(self.i2c_impl.__class__)):
                        # if using real SMBus, attempt a best-effort read - otherwise use sim
                        try:
                            if I2C_AVAILABLE and isinstance(self.i2c_impl, SMBus):
                                # proper driver needed for full gyro read; fallback to sim
                                pass
                        except Exception:
                            pass
                        if hasattr(self.i2c_impl, 'read_gyro'):
                            gx,gy,gz = self.i2c_impl.read_gyro(addr)
                            return {"gx":gx, "gy":gy, "gz":gz}

            if sensor_type in ("pressure","temp","auto"):
                if addr in (0x76, 0x77) or ("BME" in str(self.i2c_impl.__class__)):
                    if I2C_AVAILABLE and isinstance(self.i2c_impl, SMBus):
                        # full compensation omitted - users should add driver
                        try:
                            # read something, fallback
                            temp = 25.0
                            pres = 101325.0
                            return {"temp": temp, "pres": pres}
                        except Exception:
                            t,p = self.i2c_impl.read_temp_pres(addr)
                            return {"temp": t, "pres": p}
                    else:
                        t,p = self.i2c_impl.read_temp_pres(addr)
                        return {"temp": t, "pres": p}

            # ADC
            if sensor_type in ("adc","generic","auto"):
                if I2C_AVAILABLE and isinstance(self.i2c_impl, SMBus):
                    try:
                        # try read 2 bytes from address 0x48 (ADS1115)
                        data = self.i2c_impl.read_i2c_block_data(addr, 0x00, 2)
                        val = int.from_bytes(bytes(data), byteorder='big', signed=True)
                        return {"value": val}
                    except Exception:
                        # fallback to simulated ADC
                        val = self.i2c_impl.read_adc(addr)
                        return {"value": val}
                else:
                    val = self.i2c_impl.read_adc(addr)
                    return {"value": val}

        # SPI device
        if bus.startswith("spi"):
            if SPI_AVAILABLE and spidev is not None:
                try:
                    s = spidev.SpiDev()
                    # parse "spi-0.0" style
                    try:
                        parts = device_spec.get("id","spi-0.0").split("-")[-1].split(".")
                        busn, devn = int(parts[0]), int(parts[1])
                    except Exception:
                        busn, devn = 0, 0
                    s.open(busn, devn)
                    raw = s.xfer2([0x00, 0x00])
                    s.close()
                    val = (raw[0] << 8) | raw[1]
                    return {"value": val}
                except Exception:
                    sim = SpiSim()
                    sim.open(0,0)
                    raw = sim.xfer2([0x00,0x00])
                    sim.close()
                    val = (raw[0] << 8) | raw[1]
                    return {"value": val}
            else:
                sim = SpiSim()
                raw = sim.xfer2([0x00,0x00])
                val = (raw[0] << 8) | raw[1]
                return {"value": val}

        return {"error": "unsupported device type or read failed"}

# ---------- Utility functions ----------

def compute_stats_numeric(values):
    """Compute common stats for a numeric list"""
    if not values:
        return {}
    n = len(values)
    meanv = sum(values)/n
    try:
        medianv = statistics.median(values)
    except Exception:
        medianv = None
    # sample stdev
    stdevv = statistics.pstdev(values) if n==1 else statistics.stdev(values) if n>1 else 0.0
    minv = min(values)
    maxv = max(values)
    rng = maxv - minv
    # empirical resolution: smallest non-zero difference between sorted uniq values
    uniq = sorted(set(values))
    minstep = None
    if len(uniq) > 1:
        diffs = [abs(uniq[i]-uniq[i-1]) for i in range(1,len(uniq))]
        minstep = min(diffs) if diffs else 0.0
    # stdev-based resolution heuristic
    stdev_based = None
    if stdevv is not None:
        stdev_based = (stdevv * 2.0) / math.sqrt(max(1, n))
    return {
        "count": n,
        "mean": meanv,
        "median": medianv,
        "stdev": stdevv,
        "min": minv,
        "max": maxv,
        "range": rng,
        "min_step": minstep,
        "stdev_based_resolution": stdev_based
    }

# ---------- GUI Application ----------

class SensorTesterApp(ctk.CTk):
    def __init__(self):
        super().__init__()
        ctk.set_appearance_mode("dark")
        ctk.set_default_color_theme("blue")
        self.title("Sensor Tester")
        self.geometry("1100x700")
        self.service = SensorService()

        # left: controls, right: plot + results
        self.grid_rowconfigure(0, weight=1)
        self.grid_columnconfigure(1, weight=1)

        # Left frame
        lf = ctk.CTkFrame(self, width=360, corner_radius=8)
        lf.grid(row=0, column=0, sticky="nsew", padx=12, pady=12)
        lf.grid_rowconfigure(8, weight=1)
        ctk.CTkLabel(lf, text="Sensor Tester", font=ctk.CTkFont(size=18, weight="bold")).grid(row=0, column=0, padx=12, pady=(12,2), sticky="w")

        # Scan button
        self.scan_btn = ctk.CTkButton(lf, text="Scan for Devices", command=self.scan_devices)
        self.scan_btn.grid(row=1, column=0, padx=12, pady=6, sticky="ew")

        # Device list
        ctk.CTkLabel(lf, text="Devices:").grid(row=2, column=0, padx=12, pady=(6,2), sticky="w")
        self.devices_box = ctk.CTkComboBox(lf, values=[], command=self.on_device_select)
        self.devices_box.grid(row=3, column=0, padx=12, pady=2, sticky="ew")

        # Pin selection
        ctk.CTkLabel(lf, text="Pin / Interface:").grid(row=4, column=0, padx=12, pady=(8,2), sticky="w")
        self.pin_combo = ctk.CTkComboBox(lf, values=["AUTO","PIN0","PIN1","A0","A1","CS0","CS1"])
        self.pin_combo.set("AUTO")
        self.pin_combo.grid(row=5, column=0, padx=12, pady=2, sticky="ew")

        # Sensor type
        ctk.CTkLabel(lf, text="Sensor type:").grid(row=6, column=0, padx=12, pady=(8,2), sticky="w")
        # include common sensor types: ultrasonic (HC-SR04/HC-05 style), IR, gyro (MPU series)
        self.sensor_combo = ctk.CTkComboBox(lf, values=["auto","accelerometer","gyro","pressure","adc","temp","generic","serial","ultrasonic","ir","hc05"])
        self.sensor_combo.set("auto")
        self.sensor_combo.grid(row=7, column=0, padx=12, pady=2, sticky="ew")


        # Sampling controls
        samp_frame = ctk.CTkFrame(lf, corner_radius=6)
        samp_frame.grid(row=8, column=0, padx=12, pady=8, sticky="ew")
        ctk.CTkLabel(samp_frame, text="Samples:").grid(row=0, column=0, padx=6, pady=6, sticky="w")
        self.samples_entry = ctk.CTkEntry(samp_frame, width=80)
        self.samples_entry.insert(0,"200")
        self.samples_entry.grid(row=0, column=1, padx=6, pady=6)
        ctk.CTkLabel(samp_frame, text="Interval ms:").grid(row=1, column=0, padx=6, pady=6, sticky="w")
        self.interval_entry = ctk.CTkEntry(samp_frame, width=80)
        self.interval_entry.insert(0,"20")
        self.interval_entry.grid(row=1, column=1, padx=6, pady=6)

        # Run / Stop
        btn_frame = ctk.CTkFrame(lf, corner_radius=6)
        btn_frame.grid(row=9, column=0, padx=12, pady=8, sticky="ew")
        self.run_btn = ctk.CTkButton(btn_frame, text="Run Test", command=self.start_test)
        self.run_btn.grid(row=0, column=0, padx=6, pady=8, sticky="ew")
        self.stop_btn = ctk.CTkButton(btn_frame, text="Stop", fg_color="tomato", command=self.stop_test, state="disabled")
        self.stop_btn.grid(row=0, column=1, padx=6, pady=8, sticky="ew")

        # Export
        self.export_btn = ctk.CTkButton(lf, text="Export CSV (last run)", command=self.export_csv, state="disabled")
        self.export_btn.grid(row=10, column=0, padx=12, pady=6, sticky="ew")

        # Results text
        ctk.CTkLabel(lf, text="Summary:").grid(row=11, column=0, padx=12, pady=(8,2), sticky="w")
        self.summary_box = ctk.CTkTextbox(lf, width=320, height=160)
        self.summary_box.grid(row=12, column=0, padx=12, pady=(2,12), sticky="ew")

        # Right: plotting and raw data
        rf = ctk.CTkFrame(self, corner_radius=8)
        rf.grid(row=0, column=1, sticky="nsew", padx=6, pady=12)
        rf.grid_rowconfigure(1, weight=1)
        rf.grid_columnconfigure(0, weight=1)

        # Plot area
        plot_title = ctk.CTkLabel(rf, text="Live Plot", font=ctk.CTkFont(size=16, weight="bold"))
        plot_title.grid(row=0, column=0, sticky="w", padx=8, pady=(8,2))

        if plt is None:
            self.canvas_widget = ctk.CTkLabel(rf, text="matplotlib not installed — install matplotlib to see plots", text_color="gray")
            self.canvas_widget.grid(row=1, column=0, padx=8, pady=8, sticky="nsew")
            self.figure = None
            self.ax = None
        else:
            self.figure, self.ax = plt.subplots(figsize=(6,4))
            self.ax.set_title("Sensor values")
            self.ax.set_xlabel("sample")
            self.ax.set_ylabel("value")
            self.line, = self.ax.plot([], [], lw=1)
            self.canvas = FigureCanvasTkAgg(self.figure, master=rf)
            self.canvas_widget = self.canvas.get_tk_widget()
            self.canvas_widget.grid(row=1, column=0, sticky="nsew", padx=8, pady=8)

        # Raw samples text
        raw_title = ctk.CTkLabel(rf, text="Raw Samples (last 500 shown)")
        raw_title.grid(row=2, column=0, sticky="w", padx=8, pady=(4,2))
        self.raw_box = ctk.CTkTextbox(rf, height=200)
        self.raw_box.grid(row=3, column=0, padx=8, pady=(2,8), sticky="nsew")

        # Internal state
        self.current_devices = []
        self.running = False
        self._worker_thread = None
        self.latest_results = None

        # initial scan
        self.scan_devices()

    def scan_devices(self):
        self.summary_box.delete("1.0", "end")
        self.summary_box.insert("0.0", "Scanning for devices...")
        try:
            devices = self.service.detect_devices()
            self.current_devices = devices
            labels = []
            for d in devices:
                if d["bus"] == "serial":
                    labels.append(f"Serial: {d['id']}")
                elif d["bus"] == "i2c":
                    labels.append(f"I2C: {d['id']} ({','.join(hex(a) for a in d.get('addresses',[]))})")
                elif d["bus"].startswith("spi"):
                    labels.append(f"SPI: {d['id']}")
                else:
                    labels.append(str(d))
            self.devices_box.configure(values=labels)
            if labels:
                self.devices_box.set(labels[0])
            self.summary_box.delete("1.0", "end")
            self.summary_box.insert("0.0", f"Found {len(labels)} devices.")
        except Exception as e:
            self.summary_box.delete("1.0", "end")
            self.summary_box.insert("0.0", f"Error scanning devices: {e}")

    def on_device_select(self, _):
        # optional: update pin choices based on device
        pass

    def start_test(self):
        if self.running:
            return
        # figure out selected device index
        sel = self.devices_box.get()
        if not sel:
            self.summary_box.delete("1.0","end")
            self.summary_box.insert("0.0", "No device selected. Scan first.")
            return
        # map to index
        try:
            idx = self.devices_box.cget("values").index(sel)
        except Exception:
            idx = 0
        device_spec = self.current_devices[idx]
        sensor_type = self.sensor_combo.get()
        try:
            samples = int(self.samples_entry.get())
        except Exception:
            samples = 200
        try:
            interval_ms = int(self.interval_entry.get())
        except Exception:
            interval_ms = 20

        self.running = True
        self.run_btn.configure(state="disabled")
        self.stop_btn.configure(state="normal")
        self.summary_box.delete("1.0","end")
        self.summary_box.insert("0.0", f"Running {samples} samples, interval {interval_ms} ms...\n")
        # prepare plotting buffer
        self.sample_buffer = deque(maxlen=2000)
        self.raw_records = []
        # start background thread
        self._worker_thread = threading.Thread(target=self._worker_loop, args=(device_spec,sensor_type,samples,interval_ms), daemon=True)
        self._worker_thread.start()

    def stop_test(self):
        if not self.running:
            return
        self.running = False
        self.stop_btn.configure(state="disabled")
        self.run_btn.configure(state="normal")
        self.summary_box.insert("end", "\nStopping...")

    def _worker_loop(self, device_spec, sensor_type, samples, interval_ms):
        t0 = time.time()
        count = 0
        timestamps = []
        numeric_vals = []
        # if plotting present, clear axes
        if self.figure is not None and self.ax is not None:
            self.ax.cla()
            self.ax.set_title("Sensor values")
            self.ax.set_xlabel("sample")
            self.ax.set_ylabel("value")
        while self.running and count < samples:
            ts = time.time()
            try:
                res = self.service.read_once(device_spec, sensor_type, pin=self.pin_combo.get())
            except Exception as e:
                res = {"error": str(e)}
            # pick numeric
            num = None
            # possible keys: value, ax/ay/az, temp/pres
            if "error" in res:
                line = f"{count}: ERROR: {res['error']}"
                self.raw_records.append((ts, line))
                self._append_raw(line)
            else:
                if isinstance(res.get("value"), (int,float)):
                    num = float(res["value"])
                    line = f"{count}: value={num:.6g}"
                elif "ax" in res and "ay" in res and "az" in res:
                    ax,ay,az = res["ax"],res["ay"],res["az"]
                    # use magnitude for single-plot
                    mag = math.sqrt(ax*ax + ay*ay + az*az)
                    num = mag
                    line = f"{count}: ax={ax:.4f}, ay={ay:.4f}, az={az:.4f}, |a|={mag:.4f}"
                elif "temp" in res:
                    # pick temp as sample
                    num = float(res["temp"])
                    line = f"{count}: temp={num:.4f} C, pres={res.get('pres')}"
                elif "raw" in res:
                    line = f"{count}: raw={res['raw']}"
                else:
                    # fallback represent full dict
                    line = f"{count}: {res}"
                self.raw_records.append((ts, line))
                self._append_raw(line)

            if num is not None:
                numeric_vals.append(num)
                timestamps.append(ts)
                self.sample_buffer.append(num)
                # update plot
                self._update_plot()

            count += 1
            # sleep accurate interval
            elapsed = (time.time() - ts)
            wait = interval_ms/1000.0 - elapsed
            if wait > 0:
                time.sleep(wait)

        # finished or stopped
        duration = time.time() - t0
        observed_rate = len(timestamps) / duration if duration>0 else 0.0
        stats = compute_stats_numeric(numeric_vals)
        result = {
            "meta": {
                "device": device_spec,
                "sensor_type": sensor_type,
                "requested_samples": samples,
                "interval_ms": interval_ms,
                "actual_samples": len(numeric_vals),
                "duration_s": duration,
                "observed_rate_hz": observed_rate
            },
            "stats": stats,
            "raw": self.raw_records
        }
        self.latest_results = result
        # update UI (must schedule on main thread)
        self.after(0, lambda: self._on_test_finished(result))

    def _append_raw(self, line):
        # append to raw_box in UI thread
        def _do():
            self.raw_box.insert("end", line + "\n")
            # keep last 500 lines
            txt = self.raw_box.get("1.0","end").splitlines()
            if len(txt) > 500:
                keep = txt[-500:]
                self.raw_box.delete("1.0","end")
                self.raw_box.insert("0.0","\n".join(keep)+"\n")
        self.after(0,_do)

    def _update_plot(self):
        if self.figure is None:
            return
        # update line with current buffer
        y = list(self.sample_buffer)
        x = list(range(len(y)))
        # update on main thread
        def do():
            try:
                self.line.set_data(x,y)
            except Exception:
                # first-time creation
                self.line, = self.ax.plot(x,y)
            self.ax.relim()
            self.ax.autoscale_view()
            self.canvas.draw_idle()
        self.after(0, do)

    def _on_test_finished(self, result):
        self.running = False
        self.run_btn.configure(state="normal")
        self.stop_btn.configure(state="disabled")
        # pretty display
        s = result["meta"]
        st = result["stats"]
        txt = []
        # display a friendly device name if present
        dev = s.get('device') if isinstance(s.get('device'), dict) else s.get('device')
        dev_str = None
        if isinstance(dev, dict):
            dev_str = dev.get('description') or dev.get('id') or str(dev)
        else:
            dev_str = str(dev)
        txt.append(f"Device: {dev_str}")
        txt.append(f"Sensor type: {s.get('sensor_type')}")
        txt.append(f"Requested samples: {s.get('requested_samples')}, Actual numeric samples: {s.get('actual_samples')}")
        txt.append(f"Duration: {s.get('duration_s'):.3f} s, Observed rate: {s.get('observed_rate_hz'):.3f} Hz")
        if st:
            txt.append("Stats:")
            txt.append(f"  count: {st.get('count')}")
            mean_v = st.get('mean')
            txt.append(f"  mean: {mean_v:.6g}" if mean_v is not None else "  mean: N/A")
            median_v = st.get('median')
            txt.append(f"  median: {median_v:.6g}" if median_v is not None else "  median: N/A")
            stdev_v = st.get('stdev')
            txt.append(f"  stdev: {stdev_v:.6g}" if stdev_v is not None else "  stdev: N/A")
            min_v = st.get('min')
            txt.append(f"  min: {min_v:.6g}" if min_v is not None else "  min: N/A")
            max_v = st.get('max')
            txt.append(f"  max: {max_v:.6g}" if max_v is not None else "  max: N/A")
            range_v = st.get('range')
            txt.append(f"  range: {range_v:.6g}" if range_v is not None else "  range: N/A")
            txt.append(f"  min observed step: {st.get('min_step')}")
            txt.append(f"  stdev-based resolution: {st.get('stdev_based_resolution')}")
        else:
            txt.append("No numeric stats were measured.")
        self.summary_box.delete("1.0","end")
        self.summary_box.insert("0.0","\n".join(txt))
        self.export_btn.configure(state="normal")

    def export_csv(self):
        if not self.latest_results:
            return
        # Create a CSV in current dir with timestamp
        now = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"sensor_test_{now}.csv"
        try:
            with open(filename, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["timestamp","line"])
                for ts, line in self.latest_results["raw"]:
                    w.writerow([ts, line])
            self.summary_box.insert("end", f"\nExported CSV: {os.path.abspath(filename)}")
        except Exception as e:
            self.summary_box.insert("end", f"\nExport failed: {e}")

# ---------- run ----------

def main():
    app = SensorTesterApp()
    app.mainloop()

if __name__ == "__main__":
    main()
