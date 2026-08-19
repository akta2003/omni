import tkinter as tk
from tkinter import ttk, messagebox
import minimalmodbus
import serial
import serial.tools.list_ports
import time
import threading
import cv2
import json
import os
import glob
import datetime
import numpy as np
from collections import Counter
from PIL import Image, ImageTk
from ultralytics import YOLO

# =============================================================
# KONSTANTA & DEFAULT CONFIG
# =============================================================
BAUDRATE      = 9600
POLL_MS       = 50
CONFIG_FILE   = "config.json"

DEFAULT_CONFIG = {
    "slaves": [
        {"slot": 1, "id": 4,  "nama": "Omni 1", "tipe": "omni", "aktif": True},
        {"slot": 2, "id": 5,  "nama": "Omni 2", "tipe": "omni", "aktif": False},
        {"slot": 3, "id": 8,  "nama": "Omni 3", "tipe": "omni", "aktif": False},
        {"slot": 4, "id": 10, "nama": "Belt 1", "tipe": "belt", "aktif": True},
    ],
    "timing": {
        "pos_delay": 300,
        "settle_delay": 150,
        "stop_delay": 700,
        "snap_frames": 5
    },
    "routing": {
        "sempurna": [4, "LURUS"],
        "polos": [4, "KIRI"],
        "no_qr": [4, "KIRI"],
        "cacat_label": [4, "KANAN"],
        "lubang": [4, "KANAN"],
        "penyok": [4, "KANAN"],
        "berganda": [4, "KIRI"],
        "tak_terdeteksi": [4, "LURUS"]
    },
    "camera": {
        "auto_exposure": 0.25,
        "exposure": -5.0,
        "auto_wb": 0.0,
        "autofocus": 0.0,
        "focus": 0.0
    },
    "kalibrasi": {}
}

# =============================================================
# CONFIG MANAGER
# =============================================================
def load_config():
    if os.path.exists(CONFIG_FILE):
        try:
            with open(CONFIG_FILE, "r") as f:
                data = json.load(f)
            for key in DEFAULT_CONFIG:
                if key not in data:
                    data[key] = DEFAULT_CONFIG[key]
            return data
        except Exception: pass
    return json.loads(json.dumps(DEFAULT_CONFIG))

def save_config():
    try:
        cfg["slaves"] = slave_slots
        cfg["timing"] = timing_cfg
        cfg["routing"] = ROUTING_RULES
        cfg["camera"] = camera_cfg
        with open(CONFIG_FILE, "w") as f:
            json.dump(cfg, f, indent=2)
    except Exception as e:
        log(f"[CFG] Gagal simpan: {e}")

cfg         = load_config()
slave_slots = cfg["slaves"]
timing_cfg  = cfg["timing"]
camera_cfg  = cfg["camera"]

# =============================================================
# REGISTER MAP
# =============================================================
# OMNI
REG_OMNI_CMD  = 0
REG_OMNI_PROX = 1
REG_PWM1      = 4
REG_PWM2      = 5
REG_PWM3      = 6
REG_PWM4      = 7
REG_CALIB     = 11
REG_RPM1      = 12
REG_RPM2      = 13
REG_RPM3      = 14
REG_RPM4      = 15

# BELT
REG_BELT_CMD  = 5
REG_BELT_PROX = 6
REG_BELT_MODE = 7

CMD_STOP_COAST  = 0
CMD_MAJU        = 1
CMD_KANAN_ATAS  = 2
CMD_KANAN       = 3
CMD_KANAN_BAWAH = 4
CMD_MUNDUR      = 5
CMD_KIRI_BAWAH  = 6
CMD_KIRI        = 7
CMD_KIRI_ATAS   = 8
CMD_STOP_BRAKE  = 9

TURN_CMDS     = {CMD_KANAN, CMD_KIRI, CMD_KANAN_ATAS, CMD_KANAN_BAWAH, CMD_KIRI_ATAS, CMD_KIRI_BAWAH}
DIR_TO_CMD    = {"LURUS": CMD_MAJU, "KANAN": CMD_KANAN, "KIRI": CMD_KIRI}

# =============================================================
# SLAVE HELPERS
# =============================================================
SLAVE_IDS = []
last_prox = {}

def rebuild_slave_ids():
    global SLAVE_IDS, last_prox
    SLAVE_IDS = [s["id"] for s in slave_slots if s["aktif"]]
    last_prox = {sid: 0 for sid in SLAVE_IDS}

rebuild_slave_ids()

def get_active_slots():  return [s for s in slave_slots if s["aktif"]]
def get_omni_slots():    return [s for s in slave_slots if s["aktif"] and s["tipe"] == "omni"]
def get_belt_slots():    return [s for s in slave_slots if s["aktif"] and s["tipe"] == "belt"]
def get_slot_by_id(sid):
    for s in slave_slots:
        if s["id"] == sid: return s
    return None

def get_omni_path(target_id):
    omnis = get_omni_slots()
    path = []
    for o in omnis:
        path.append(o["id"])
        if o["id"] == target_id:
            break
    return path if target_id in path else [target_id]

# =============================================================
# ROUTING RULES
# =============================================================
ROUTING_RULES = cfg["routing"]

# =============================================================
# GLOBALS
# =============================================================
master           = None
robots           = {}
modbus_lock      = threading.Lock()
modbus_connected = False

auto_mode        = False
auto_state       = "IDLE"
sm_timer         = 0
plan_target      = None
plan_dir         = None

# Kamera / YOLO / Snap
camera_running       = True
camera_active        = False
latest_frame_rgb     = None
latest_raw_frame     = None
frame_lock           = threading.Lock()
camera_index         = 0
yolo_model           = None
yolo_model_path      = ""
cam_settings_changed = False 

snap_active         = False
snap_count          = 0
snap_history        = []
detection_only_mode = False

# Kalibrasi Globals
calib_polling_active = False
calib_live_data      = {} 

# =============================================================
# UTILS & THREADS
# =============================================================
def scan_com_ports():
    ports = serial.tools.list_ports.comports()
    result = [p.device for p in sorted(ports)]
    return result if result else ["(tidak ada COM)"]

def scan_yolo_models():
    pts = glob.glob("*.pt")
    return pts if pts else ["(tidak ada .pt)"]

def load_yolo_model():
    global yolo_model
    if not yolo_model_path or not os.path.exists(yolo_model_path):
        log(f"[YOLO] File tidak ditemukan: '{yolo_model_path}'"); return
    try:
        log(f"[YOLO] Memuat: {yolo_model_path} ...")
        yolo_model = YOLO(yolo_model_path)
        log(f"[YOLO] Model berhasil dimuat!")
    except Exception as e:
        log(f"[YOLO ERR] {e}")

def reload_yolo_model(new_path):
    global yolo_model, yolo_model_path
    yolo_model      = None
    yolo_model_path = new_path
    threading.Thread(target=load_yolo_model, daemon=True).start()

def camera_capture_thread():
    global latest_raw_frame, camera_running, camera_active, camera_index
    global cam_settings_changed
    
    while camera_running:
        if not camera_active:
            time.sleep(0.1); continue
        
        cap = cv2.VideoCapture(camera_index, cv2.CAP_DSHOW)
        if not cap.isOpened():
            camera_active = False
            root.after(0, lambda: btn_cam_ctrl.config(text="▶ Start Kamera", bg="#388E3C", fg="white"))
            time.sleep(1); continue
            
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)
        
        # Inisialisasi pengaturan dari config
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, camera_cfg.get("auto_exposure", 0.25)) 
        cap.set(cv2.CAP_PROP_EXPOSURE, camera_cfg.get("exposure", -5.0)) 
        cap.set(cv2.CAP_PROP_AUTO_WB, camera_cfg.get("auto_wb", 0.0))
        cap.set(cv2.CAP_PROP_AUTOFOCUS, camera_cfg.get("autofocus", 0.0))
        cap.set(cv2.CAP_PROP_FOCUS, camera_cfg.get("focus", 0.0))
        cam_settings_changed = False
        
        while camera_running and camera_active:
            if cam_settings_changed:
                cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, camera_cfg.get("auto_exposure", 0.25)) 
                cap.set(cv2.CAP_PROP_EXPOSURE, camera_cfg.get("exposure", -5.0)) 
                cap.set(cv2.CAP_PROP_AUTO_WB, camera_cfg.get("auto_wb", 0.0))
                cap.set(cv2.CAP_PROP_AUTOFOCUS, camera_cfg.get("autofocus", 0.0))
                cap.set(cv2.CAP_PROP_FOCUS, camera_cfg.get("focus", 0.0))
                cam_settings_changed = False
                
            ret, frame = cap.read()
            if ret:
                with frame_lock: latest_raw_frame = frame
            else: time.sleep(0.01)
        cap.release()
        with frame_lock: latest_raw_frame = None
        time.sleep(0.1)

_last_yolo_time = 0

def yolo_inference_thread():
    global latest_frame_rgb, camera_running, yolo_model, latest_raw_frame
    global snap_active, snap_count, snap_history, auto_state
    global _last_yolo_time

    while camera_running:
        if latest_raw_frame is None:
            blank = np.zeros((360, 640, 3), dtype=np.uint8)
            msg   = "KAMERA OFF" if not camera_active else "MENGHUBUNGKAN..."
            cv2.putText(blank, msg, (180, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (150,150,150), 2)
            with frame_lock: latest_frame_rgb = blank
            time.sleep(0.1); continue

        with frame_lock: frame = latest_raw_frame.copy()

        now = time.time()
        if now - _last_yolo_time < 0.1:
            time.sleep(0.02); continue
        _last_yolo_time = now

        if yolo_model is None:
            display = frame.copy()
            cv2.putText(display, "Model YOLO belum dimuat...", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 220, 220), 2)
            with frame_lock: latest_frame_rgb = cv2.cvtColor(display, cv2.COLOR_BGR2RGB)
            time.sleep(0.05); continue

        if detection_only_mode or snap_active:
            results   = yolo_model.predict(frame, verbose=False, conf=0.6, imgsz=640)
            annotated = results[0].plot()
            
            frame_detections = set()
            for box in results[0].boxes:
                cls_name = yolo_model.names[int(box.cls[0])].lower().replace(" ", "_")
                conf     = float(box.conf[0])
                if conf > 0.7:
                    frame_detections.add(cls_name)

            if snap_active and snap_count < timing_cfg["snap_frames"]:
                if frame_detections: 
                    snap_history.extend(list(frame_detections))
                
                snap_count += 1
                cv2.putText(annotated, f"SNAP {snap_count}/{timing_cfg['snap_frames']}...", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                
                if snap_count >= timing_cfg["snap_frames"]:
                    snap_active = False
                    auto_state = "PROCESS_VOTE"
            else:
                cv2.putText(annotated, "DETEKSI SAJA", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)

            with frame_lock: latest_frame_rgb = cv2.cvtColor(annotated, cv2.COLOR_BGR2RGB)
        else:
            display = frame.copy()
            cv2.putText(display, "SISTEM SIAP", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 165, 0), 2)
            with frame_lock: latest_frame_rgb = cv2.cvtColor(display, cv2.COLOR_BGR2RGB)

def update_gui_video():
    with frame_lock: frame = latest_frame_rgb
    if frame is not None:
        try:
            img   = Image.fromarray(frame)
            imgtk = ImageTk.PhotoImage(image=img)
            lbl_video.imgtk = imgtk
            lbl_video.configure(image=imgtk)
        except Exception: pass
    root.after(50, update_gui_video)

# =============================================================
# MODBUS PROTOCOL UTILS
# =============================================================
def safe_modbus_setup(inst):
    inst.serial.baudrate = BAUDRATE; inst.serial.bytesize = 8
    inst.serial.parity   = serial.PARITY_NONE; inst.serial.stopbits = 1
    inst.serial.timeout  = 0.50; inst.mode = minimalmodbus.MODE_RTU
    inst.clear_buffers_before_each_transaction = True
    inst.close_port_after_each_call            = False

def mb_write_reg(sid, reg, value):
    if master is None: return
    try:
        with modbus_lock:
            master.address = sid
            master.write_register(reg, int(value), 0, functioncode=6)
    except: pass

def mb_read_reg(sid, reg):
    if master is None: return 0
    try:
        with modbus_lock:
            master.address = sid
            return int(master.read_register(reg, 0, functioncode=3))
    except: return 0

def mb_read_regs(sid, start_reg, count):
    if master is None: return [0]*count
    try:
        with modbus_lock:
            master.address = sid
            return master.read_registers(start_reg, count, functioncode=3)
    except: return [0]*count

def write_cmd(sid, cmd):
    slot = get_slot_by_id(sid)
    if not slot: return
    reg = REG_BELT_CMD if slot["tipe"] == "belt" else REG_OMNI_CMD
    mb_write_reg(sid, reg, cmd)
    if sid in cmd_lbls:
        root.after(0, lambda s=sid, c=cmd: cmd_lbls[s].config(
            text=f"CMD:{c}", fg=("#9C27B0" if c in TURN_CMDS else "#212121")))

def read_prox(sid):
    slot = get_slot_by_id(sid)
    if not slot: return 0
    reg = REG_BELT_PROX if slot["tipe"] == "belt" else REG_OMNI_PROX
    return 1 if mb_read_reg(sid, reg) != 0 else 0

def log(msg):
    def _log():
        log_box.config(state="normal")
        log_box.insert("end", msg + "\n")
        log_box.see("end")
        lines = int(log_box.index("end-1c").split(".")[0])
        if lines > 200: log_box.delete("1.0", f"{lines-200}.0")
        log_box.config(state="disabled")
    root.after(0, _log)

# =============================================================
# AUTO / MANUAL STATE MACHINE
# =============================================================
def toggle_auto():
    global auto_mode, auto_state, sm_timer
    if not get_belt_slots():
        messagebox.showwarning("Error", "Minimal 1 Belt Slave harus aktif!"); return
    
    auto_mode = not auto_mode
    if auto_mode:
        auto_state = "STANDBY"
        sm_timer = time.time()
        btn_auto.config(text="⏹ STOP AUTO", bg="#C62828", fg="white")
        log("=== MODE AUTO AKTIF ===")
        write_cmd(get_belt_slots()[0]["id"], CMD_MAJU)
    else:
        auto_state = "IDLE"
        btn_auto.config(text="▶ START AUTO SORTIR", bg="#388E3C", fg="white")
        stop_all()
        log("=== MODE AUTO MATI ===")

def start_plan():
    global auto_mode, auto_state, plan_target, plan_dir, sm_timer
    if master is None or auto_mode: return
    target    = int(combo_target.get())
    direction = combo_dir.get().strip().upper()
    if target not in SLAVE_IDS: return

    plan_target = target
    plan_dir    = DIR_TO_CMD.get(direction, CMD_MAJU)
    auto_state  = "PROCESS_MANUAL"
    sm_timer    = time.time()
    log(f"[MANUAL] Pindah box ke Omni {target} arah {direction}")

def stop_all():
    global auto_state
    auto_state = "IDLE"
    if master:
        for sid in SLAVE_IDS:
            write_cmd(sid, CMD_STOP_COAST)
    root.after(0, lambda: lbl_plan.config(text="SISTEM BERHENTI (STOP ALL)", fg="#C62828"))
    log("[!] STOP ALL: Command 0 dikirim ke semua slave")

def update_prox_gui(sid, val):
    if sid in prox_lbls:
        prox_lbls[sid].config(text=f"PROX:{'●' if val==1 else '○'}", fg=("#388E3C" if val == 1 else "#9E9E9E"))

def modbus_polling_thread():
    global last_prox, auto_state, sm_timer
    global plan_target, plan_dir, snap_active, snap_count, snap_history
    
    while True:
        if not modbus_connected or master is None:
            time.sleep(POLL_MS / 1000.0); continue
            
        belts = get_belt_slots()
        omnis = get_omni_slots()
        belt_id = belts[0]["id"] if belts else None

        prox_now = {}
        for sid in SLAVE_IDS:
            val = read_prox(sid)
            prox_now[sid] = val
            root.after(0, update_prox_gui, sid, val)
        
        if auto_mode and belt_id:
            if auto_state == "STANDBY":
                root.after(0, lambda: lbl_plan.config(text="[AUTO] STANDBY - Belt Jalan", fg="#388E3C"))
                if prox_now.get(belt_id, 0) == 1:
                    auto_state = "POSITIONING"
                    sm_timer = time.time()
                    
            elif auto_state == "POSITIONING":
                root.after(0, lambda: lbl_plan.config(text="[AUTO] Menyesuaikan Posisi...", fg="#E65100"))
                if time.time() - sm_timer >= (timing_cfg["pos_delay"] / 1000.0):
                    write_cmd(belt_id, CMD_STOP_COAST)
                    auto_state = "SETTLING"
                    sm_timer = time.time()
                    
            elif auto_state == "SETTLING":
                if time.time() - sm_timer >= (timing_cfg["settle_delay"] / 1000.0):
                    snap_count = 0
                    snap_history = []
                    snap_active = True
                    auto_state = "SNAP_VOTE"
                    sm_timer = time.time() 
            
            elif auto_state == "SNAP_VOTE":
                root.after(0, lambda: lbl_plan.config(text="[AUTO] Memotret Box...", fg="#E65100"))
                if time.time() - sm_timer > 2.0:
                    snap_active = False
                    log("[WARN] Timeout Kamera! Paksa lanjut.")
                    auto_state = "PROCESS_VOTE"
            
            elif auto_state == "PROCESS_VOTE":
                if not snap_history: 
                    final_res = "tak_terdeteksi"
                else: 
                    counts = Counter(snap_history)
                    min_frames = max(1, timing_cfg["snap_frames"] // 2)
                    valid_classes = [cls for cls, count in counts.items() if count >= min_frames]
                    
                    if not valid_classes:
                        final_res = "tak_terdeteksi"
                    elif "cacat_label" in valid_classes or "lubang" in valid_classes or "penyok" in valid_classes:
                        defects = [c for c in valid_classes if c in ["cacat_label", "lubang", "penyok"]]
                        
                        if len(defects) > 1 or "qr" not in valid_classes:
                            final_res = "berganda" 
                        else:
                            final_res = defects[0]
                            
                    elif "good" in valid_classes and "qr" in valid_classes:
                        final_res = "sempurna"
                    elif "good" in valid_classes and "qr" not in valid_classes:
                        final_res = "no_qr"
                    elif "polos" in valid_classes:
                        final_res = "polos"
                    else:
                        final_res = "cacat_label"
                
                plan_target, arah = ROUTING_RULES.get(final_res, [omnis[0]["id"] if omnis else 4, "LURUS"])
                plan_dir = DIR_TO_CMD.get(arah, CMD_MAJU)
                
                log(f"[AI] Terbaca: {valid_classes if snap_history else 'kosong'} -> Final: {final_res} -> Omni {plan_target} (Arah: {arah})")
                
                rute_omni = get_omni_path(plan_target)
                for sid in rute_omni:
                    write_cmd(sid, CMD_MAJU)
                
                write_cmd(belt_id, CMD_MAJU) 
                auto_state = "WAIT_OMNI_ENTER"
                sm_timer = time.time()
            
            elif auto_state == "WAIT_OMNI_ENTER":
                root.after(0, lambda: lbl_plan.config(text=f"[AUTO] Transfer ke Omni {plan_target}...", fg="#1565C0"))
                
                fast_prox = read_prox(plan_target)
                prox_now[plan_target] = fast_prox
                root.after(0, update_prox_gui, plan_target, fast_prox)
                
                if fast_prox == 1:
                    if plan_dir != CMD_MAJU:
                        write_cmd(plan_target, plan_dir)
                    
                    for sid in get_omni_path(plan_target):
                        if sid != plan_target:
                            write_cmd(sid, CMD_STOP_COAST)
                            
                    auto_state = "WAIT_OMNI_EXIT"
                    sm_timer = time.time()
                
                elif time.time() - sm_timer > 5.0:
                    log(f"[ERR] Barang nyangkut! Reset dari Omni {plan_target}.")
                    for sid in get_omni_path(plan_target):
                        write_cmd(sid, CMD_STOP_COAST)
                    write_cmd(belt_id, CMD_MAJU)
                    auto_state = "STANDBY"
            
            elif auto_state == "WAIT_OMNI_EXIT":
                root.after(0, lambda: lbl_plan.config(text=f"[AUTO] Omni {plan_target} membuang box...", fg="#9C27B0"))
                if prox_now.get(plan_target, 0) == 0:
                    sm_timer = time.time()
                    auto_state = "WAIT_STOP_DELAY"
            
            elif auto_state == "WAIT_STOP_DELAY":
                if time.time() - sm_timer >= (timing_cfg["stop_delay"] / 1000.0):
                    rute_omni = get_omni_path(plan_target)
                    for sid in rute_omni:
                        write_cmd(sid, CMD_STOP_COAST)
                    
                    write_cmd(belt_id, CMD_MAJU)
                    auto_state = "STANDBY"
                    log(f"[AUTO] Siklus selesai.")

        elif not auto_mode and auto_state == "PROCESS_MANUAL":
            rute_omni = get_omni_path(plan_target)
            for sid in rute_omni:
                write_cmd(sid, CMD_MAJU)
            if belt_id: write_cmd(belt_id, CMD_MAJU)
            auto_state = "WAIT_OMNI_ENTER"
            sm_timer = time.time()
            
        elif not auto_mode and auto_state == "WAIT_OMNI_ENTER":
            root.after(0, lambda: lbl_plan.config(text=f"[MANUAL] Transfer ke Omni {plan_target}...", fg="#1565C0"))
            
            fast_prox = read_prox(plan_target)
            prox_now[plan_target] = fast_prox
            root.after(0, update_prox_gui, plan_target, fast_prox)
            
            if fast_prox == 1:
                if plan_dir != CMD_MAJU: 
                    write_cmd(plan_target, plan_dir)
                    
                for sid in get_omni_path(plan_target):
                    if sid != plan_target:
                        write_cmd(sid, CMD_STOP_COAST)
                        
                auto_state = "WAIT_OMNI_EXIT"
                sm_timer = time.time()
            elif time.time() - sm_timer > 5.0:
                auto_state = "IDLE"
                log("[ERR] Manual transfer timeout.")
                stop_all()
                
        elif not auto_mode and auto_state == "WAIT_OMNI_EXIT":
            if prox_now.get(plan_target, 0) == 0:
                sm_timer = time.time()
                auto_state = "WAIT_STOP_DELAY"
                
        elif not auto_mode and auto_state == "WAIT_STOP_DELAY":
            if time.time() - sm_timer >= (timing_cfg["stop_delay"] / 1000.0):
                rute_omni = get_omni_path(plan_target)
                for sid in rute_omni:
                    write_cmd(sid, CMD_STOP_COAST)
                auto_state = "IDLE"
                root.after(0, lambda: lbl_plan.config(text="Sistem Siap", fg="#388E3C"))

        last_prox = prox_now
        time.sleep(POLL_MS / 1000.0)

# =============================================================
# KALIBRASI MOTOR POLLING & GUI
# =============================================================
def calib_polling_thread(sids_to_poll, on_update):
    global calib_polling_active, calib_live_data
    while calib_polling_active:
        for sid in sids_to_poll:
            if not calib_polling_active: break
            try:
                pwm_data = mb_read_regs(sid, REG_PWM1, 4)
                rpm_data = mb_read_regs(sid, REG_RPM1, 4)
                calib_reg = mb_read_reg(sid, REG_CALIB)
                calib_live_data[sid] = {
                    "pwm": list(pwm_data), "rpm": list(rpm_data),
                    "done": calib_reg == 0, "error": False
                }
            except Exception as e:
                if sid not in calib_live_data:
                    calib_live_data[sid] = {"pwm": [0,0,0,0], "rpm": [0,0,0,0], "done": False, "error": True}
                else:
                    calib_live_data[sid]["error"] = True
            time.sleep(0.05)
        root.after(0, on_update)
        time.sleep(0.2)

def open_calib_dialog():
    global calib_polling_active, calib_live_data
    dialog = tk.Toplevel(root)
    dialog.title("⚙  Kalibrasi Motor")
    dialog.geometry("700x580")
    dialog.resizable(False, False)
    dialog.grab_set()

    hdr = tk.Frame(dialog, bg="#1A237E"); hdr.pack(fill="x")
    tk.Label(hdr, text="  ⚙  Kalibrasi Motor — RPM Target", font=("Consolas", 12, "bold"), bg="#1A237E", fg="white").pack(side="left", pady=8)
    body = tk.Frame(dialog, bg="#F5F5F5"); body.pack(fill="both", expand=True, padx=12, pady=8)

    frm_setup = tk.LabelFrame(body, text="Setup Kalibrasi", bg="#F5F5F5", font=("Consolas", 9, "bold"), padx=8, pady=6)
    frm_setup.pack(fill="x", pady=(0, 6))
    tk.Label(frm_setup, text="RPM Target (100–200):", bg="#F5F5F5", font=("Consolas", 9)).grid(row=0, column=0, sticky="w", padx=4)
    var_rpm = tk.IntVar(value=150)
    tk.Spinbox(frm_setup, from_=100, to=200, textvariable=var_rpm, width=6, font=("Consolas", 11, "bold"), justify="center").grid(row=0, column=1, padx=8, pady=4)
    tk.Label(frm_setup, text="Pilih Omni:", bg="#F5F5F5", font=("Consolas", 9)).grid(row=1, column=0, sticky="w", padx=4, pady=4)
    
    slave_vars = {}
    frm_slaves = tk.Frame(frm_setup, bg="#F5F5F5")
    frm_slaves.grid(row=1, column=1, columnspan=3, sticky="w")
    for slot in get_omni_slots():
        sid  = slot["id"]
        var  = tk.BooleanVar(value=True)
        slave_vars[sid] = var
        tk.Checkbutton(frm_slaves, text=f"{slot['nama']} (C{sid})", variable=var, bg="#F5F5F5", font=("Consolas", 9)).pack(side="left", padx=4)

    frm_monitor = tk.LabelFrame(body, text="Monitor Live", bg="#F5F5F5", font=("Consolas", 9, "bold"), padx=8, pady=6)
    frm_monitor.pack(fill="both", expand=True, pady=(0, 6))
    frm_sel = tk.Frame(frm_monitor, bg="#F5F5F5"); frm_sel.pack(fill="x", pady=(0, 4))
    tk.Label(frm_sel, text="Pantau slave:", bg="#F5F5F5", font=("Consolas", 9)).pack(side="left")
    active_ids = [str(s["id"]) for s in get_omni_slots()]
    var_monitor_sel = tk.StringVar(value=active_ids[0] if active_ids else "")
    ttk.Combobox(frm_sel, textvariable=var_monitor_sel, values=active_ids, state="readonly", width=6).pack(side="left", padx=6)

    cols = ("Motor", "PWM", "RPM", "Status")
    tree = ttk.Treeview(frm_monitor, columns=cols, show="headings", height=4)
    for col in cols:
        tree.heading(col, text=col)
        tree.column(col, width=(80 if col != "Status" else 160), anchor="center")
    tree.pack(fill="x", pady=4)
    lbl_calib_status = tk.Label(frm_monitor, text="Belum mulai.", bg="#F5F5F5", font=("Consolas", 9), fg="#757575")
    lbl_calib_status.pack(anchor="w")

    frm_pwm = tk.LabelFrame(body, text="PWM Manual (kirim setelah kalibrasi)", bg="#F5F5F5", font=("Consolas", 9, "bold"), padx=8, pady=6)
    frm_pwm.pack(fill="x", pady=(0, 6))
    pwm_vars = [tk.IntVar(value=200) for _ in range(4)]
    var_pwm_slave = tk.StringVar(value=active_ids[0] if active_ids else "")
    tk.Label(frm_pwm, text="Slave:", bg="#F5F5F5", font=("Consolas", 9)).grid(row=0, column=0, sticky="w")
    ttk.Combobox(frm_pwm, textvariable=var_pwm_slave, values=active_ids, state="readonly", width=6).grid(row=0, column=1, padx=4)
    
    for i, (lbl, var) in enumerate(zip(["M1", "M2", "M3", "M4"], pwm_vars)):
        tk.Label(frm_pwm, text=f"{lbl}:", bg="#F5F5F5", font=("Consolas", 9)).grid(row=0, column=2 + i*2, padx=(8,0), sticky="e")
        tk.Spinbox(frm_pwm, from_=0, to=255, textvariable=var, width=4, font=("Consolas", 9), justify="center").grid(row=0, column=3 + i*2, padx=2)

    def send_pwm_manual():
        if master is None: messagebox.showwarning("Error", "Belum Connect Modbus!"); return
        try:
            sid = int(var_pwm_slave.get())
            for reg, var in zip([REG_PWM1, REG_PWM2, REG_PWM3, REG_PWM4], pwm_vars):
                mb_write_reg(sid, reg, max(0, min(255, var.get()))); time.sleep(0.02)
            messagebox.showinfo("Berhasil", f"PWM dikirim ke C{sid}")
        except Exception as e: log(f"[PWM ERR] {e}")

    tk.Button(frm_pwm, text="Kirim PWM", command=send_pwm_manual, bg="#37474F", fg="white", font=("Consolas", 9, "bold"), padx=8).grid(row=0, column=10, padx=(12, 0))

    def update_monitor_table():
        try: sid = int(var_monitor_sel.get())
        except: return
        tree.delete(*tree.get_children())
        data = calib_live_data.get(sid)
        if not data:
            lbl_calib_status.config(text="Belum ada data.", fg="#757575"); return

        done, error = data.get("done", False), data.get("error", False)
        pwms, rpms = data.get("pwm", [0]*4), data.get("rpm", [0]*4)
        for i in range(4):
            status = "✅ Selesai" if done else ("❌ Error" if error else "🔄 Berjalan")
            tree.insert("", "end", values=(f"M{i+1}", pwms[i], rpms[i], status))
        
        if done: lbl_calib_status.config(text=f"✅ C{sid} kalibrasi selesai!", fg="#2E7D32")
        elif error: lbl_calib_status.config(text=f"❌ C{sid} tidak merespons.", fg="#C62828")
        else: lbl_calib_status.config(text=f"⏳ C{sid} sedang kalibrasi...", fg="#E65100")

    frm_btn = tk.Frame(dialog, bg="#EEEEEE"); frm_btn.pack(fill="x", padx=12, pady=8)
    btn_mulai = tk.Button(frm_btn, text="▶  Mulai Kalibrasi", bg="#1A237E", fg="white", font=("Consolas", 10, "bold"), padx=12)
    btn_mulai.pack(side="left", padx=4)
    btn_stop_calib = tk.Button(frm_btn, text="⏹  Stop", bg="#C62828", fg="white", font=("Consolas", 10, "bold"), padx=12, state="disabled")
    btn_stop_calib.pack(side="left", padx=4)
    tk.Button(frm_btn, text="Tutup", command=dialog.destroy, font=("Consolas", 10), padx=12).pack(side="right", padx=4)

    def do_mulai():
        global calib_polling_active, calib_live_data
        if master is None: messagebox.showwarning("Error", "Belum Connect!"); return
        rpm = var_rpm.get()
        sids_dipilih = [sid for sid, var in slave_vars.items() if var.get()]
        if not sids_dipilih: return
        
        calib_polling_active = False 
        
        calib_live_data = {}
        for sid in sids_dipilih:
            try: mb_write_reg(sid, REG_CALIB, rpm)
            except: pass

        time.sleep(0.5)
        
        for sid in sids_dipilih:
            calib_live_data[sid] = {"pwm": [0]*4, "rpm": [0]*4, "done": False, "error": False}

        calib_polling_active = True
        threading.Thread(target=calib_polling_thread, args=(sids_dipilih, update_monitor_table), daemon=True).start()
        btn_mulai.config(state="disabled"); btn_stop_calib.config(state="normal")
        lbl_calib_status.config(text="⏳ Kalibrasi berjalan...", fg="#E65100")

        def check_done():
            if all(calib_live_data.get(sid, {}).get("done", False) for sid in sids_dipilih):
                btn_mulai.config(state="normal"); btn_stop_calib.config(state="disabled")
                log("[CALIB] Semua slave selesai dikalibrasi.")
            else: dialog.after(1000, check_done)
        dialog.after(1000, check_done)

    def do_stop_calib():
        global calib_polling_active
        calib_polling_active = False
        if master:
            for sid in calib_live_data.keys():
                try: mb_write_reg(sid, REG_CALIB, 0)
                except: pass
        btn_mulai.config(state="normal"); btn_stop_calib.config(state="disabled")
        lbl_calib_status.config(text="Dihentikan manual.", fg="#757575")

    def on_dialog_close():
        global calib_polling_active
        calib_polling_active = False; dialog.destroy()

    btn_mulai.config(command=do_mulai); btn_stop_calib.config(command=do_stop_calib)
    dialog.protocol("WM_DELETE_WINDOW", on_dialog_close)

# =============================================================
# POPUP SETTINGS LAINNYA
# =============================================================
def open_slave_config_dialog():
    dialog = tk.Toplevel(root)
    dialog.title("⚙  Konfigurasi Slave")
    dialog.geometry("540x320")
    dialog.resizable(False, False)
    dialog.grab_set()

    hdr = tk.Frame(dialog, bg="#1A237E"); hdr.pack(fill="x")
    tk.Label(hdr, text="  ⚙  Konfigurasi Slave", font=("Consolas", 11, "bold"), bg="#1A237E", fg="white").pack(side="left", pady=8)

    frm_grid = tk.Frame(dialog, padx=12); frm_grid.pack(fill="x", pady=8)
    headers = ["Slot", "Aktif", "ID (1–247)", "Nama", "Tipe"]
    for c, h in enumerate(headers):
        tk.Label(frm_grid, text=h, font=("Consolas", 9, "bold"), bg="#E8EAF6").grid(row=0, column=c, padx=3, pady=2, sticky="ew")

    row_vars = []
    for i, slot in enumerate(slave_slots):
        var_aktif = tk.BooleanVar(value=slot["aktif"])
        var_id    = tk.StringVar(value=str(slot["id"]))
        var_nama  = tk.StringVar(value=slot["nama"])
        var_tipe  = tk.StringVar(value=slot["tipe"])
        row_vars.append({"aktif": var_aktif, "id": var_id, "nama": var_nama, "tipe": var_tipe})
        row = i + 1
        tk.Label(frm_grid, text=f"Slot {i+1}", font=("Consolas", 9)).grid(row=row, column=0, padx=3, pady=3)
        tk.Checkbutton(frm_grid, variable=var_aktif).grid(row=row, column=1, padx=3)
        tk.Entry(frm_grid, textvariable=var_id, width=8, font=("Consolas", 9), justify="center").grid(row=row, column=2, padx=3)
        tk.Entry(frm_grid, textvariable=var_nama, width=12, font=("Consolas", 9)).grid(row=row, column=3, padx=3)
        ttk.Combobox(frm_grid, textvariable=var_tipe, values=["omni","belt"], width=6, state="readonly").grid(row=row, column=4, padx=3)

    def do_save():
        for i, rv in enumerate(row_vars):
            slave_slots[i]["id"]    = int(rv["id"].get())
            slave_slots[i]["nama"]  = rv["nama"].get().strip()
            slave_slots[i]["tipe"]  = rv["tipe"].get()
            slave_slots[i]["aktif"] = rv["aktif"].get()
        save_config()
        rebuild_slave_ids()
        refresh_live_status_panel()
        refresh_target_dropdowns()
        dialog.destroy()
        log(f"[SLAVE] Config disimpan. Aktif: {SLAVE_IDS}")

    frm_btn = tk.Frame(dialog); frm_btn.pack(pady=10)
    tk.Button(frm_btn, text="💾  Simpan", command=do_save, bg="#1A237E", fg="white", font=("Consolas", 10, "bold"), padx=12).pack(side="left", padx=6)

def open_timing_dialog():
    dialog = tk.Toplevel(root)
    dialog.title("⏱ Timing & Delay")
    dialog.geometry("340x260")
    dialog.resizable(False, False)
    dialog.grab_set()

    hdr = tk.Frame(dialog, bg="#00838F"); hdr.pack(fill="x")
    tk.Label(hdr, text="  ⏱  Pengaturan Waktu (Delay)", font=("Consolas", 11, "bold"), bg="#00838F", fg="white").pack(side="left", pady=8)

    frm = tk.Frame(dialog, padx=20, pady=10); frm.pack(fill="both")
    
    var_pos = tk.IntVar(value=timing_cfg["pos_delay"])
    var_set = tk.IntVar(value=timing_cfg["settle_delay"])
    var_stp = tk.IntVar(value=timing_cfg["stop_delay"])
    var_snp = tk.IntVar(value=timing_cfg["snap_frames"])

    def add_row(lbl, var):
        r = tk.Frame(frm); r.pack(fill="x", pady=5)
        tk.Label(r, text=lbl, font=("Consolas", 9)).pack(side="left")
        tk.Entry(r, textvariable=var, width=8, justify="center").pack(side="right")

    add_row("Posisi Meluncur (ms) :", var_pos)
    add_row("Jeda Diam Difoto (ms):", var_set)
    add_row("Jeda Tunggu Jatuh (ms):", var_stp)
    add_row("Jumlah Snap Frame    :", var_snp)

    def do_save():
        timing_cfg["pos_delay"] = var_pos.get()
        timing_cfg["settle_delay"] = var_set.get()
        timing_cfg["stop_delay"] = var_stp.get()
        timing_cfg["snap_frames"] = var_snp.get()
        save_config()
        log("[TIMING] Pengaturan Waktu disimpan.")
        dialog.destroy()

    frm_btn = tk.Frame(dialog); frm_btn.pack(pady=10)
    tk.Button(frm_btn, text="💾  Simpan", command=do_save, bg="#00838F", fg="white", font=("Consolas", 10, "bold"), padx=12).pack()

def open_routing_dialog():
    dialog = tk.Toplevel(root)
    dialog.title("🔀  Konfigurasi Rute Otomatis")
    dialog.geometry("480x420")
    dialog.resizable(False, False)
    dialog.grab_set()

    hdr = tk.Frame(dialog, bg="#1B5E20"); hdr.pack(fill="x")
    tk.Label(hdr, text="  🔀  Konfigurasi Rute Otomatis", font=("Consolas", 11, "bold"), bg="#1B5E20", fg="white").pack(side="left", pady=8)

    frm_grid = tk.Frame(dialog, padx=12); frm_grid.pack(fill="x", pady=4)
    headers = ["Kondisi Produk", "Target Slave", "Arah"]
    for c, h in enumerate(headers):
        tk.Label(frm_grid, text=h, font=("Consolas", 9, "bold"), bg="#E8F5E9").grid(row=0, column=c, padx=4, pady=3, sticky="ew")

    routes_info = [
        ("sempurna",       "✅  Sempurna (Good+QR)"),
        ("polos",          "📦  Polos / Kosong"),
        ("no_qr",          "🔖  Tanpa QR"),
        ("cacat_label",    "🏷  Cacat Label"),
        ("lubang",         "🕳  Berlubang"),
        ("penyok",         "💥  Penyok"),
        ("berganda",       "⚠  Cacat Ganda"),
        ("tak_terdeteksi", "❓  Tak Terdeteksi (Kosong)"),
    ]

    all_omni_ids = [str(s["id"]) for s in get_omni_slots()]
    route_vars = {}

    for i, (key, label_text) in enumerate(routes_info):
        tk.Label(frm_grid, text=label_text, font=("Consolas", 9), anchor="w").grid(row=i+1, column=0, sticky="w", pady=3, padx=4)
        
        cur_setting = ROUTING_RULES.get(key, [int(all_omni_ids[0]) if all_omni_ids else 4, "LURUS"])
        cur_target = str(cur_setting[0])
        cur_arah   = cur_setting[1]
        
        var_t = tk.StringVar(value=cur_target)
        var_d = tk.StringVar(value=cur_arah)
        route_vars[key] = {"target": var_t, "arah": var_d}

        ttk.Combobox(frm_grid, textvariable=var_t, values=all_omni_ids, width=5, state="readonly").grid(row=i+1, column=1, padx=4, pady=3)
        ttk.Combobox(frm_grid, textvariable=var_d, values=["LURUS","KANAN","KIRI"], width=8, state="readonly").grid(row=i+1, column=2, padx=4, pady=3)

    def do_save_routing():
        for key, vars in route_vars.items():
            try: ROUTING_RULES[key] = [int(vars["target"].get()), vars["arah"].get()]
            except: pass
        save_config()
        log("[ROUTING] Konfigurasi rute disimpan (JSON).")
        dialog.destroy()

    frm_btn = tk.Frame(dialog); frm_btn.pack(pady=10)
    tk.Button(frm_btn, text="💾  Simpan", command=do_save_routing, bg="#1B5E20", fg="white", font=("Consolas", 10, "bold"), padx=12).pack(side="left", padx=6)

def open_camera_settings_dialog():
    dialog = tk.Toplevel(root)
    dialog.title("📷 Konfigurasi Kamera")
    dialog.geometry("380x380")
    dialog.resizable(False, False)
    dialog.grab_set()

    hdr = tk.Frame(dialog, bg="#E65100")
    hdr.pack(fill="x")
    tk.Label(hdr, text="  📷  Pengaturan Lensa & Warna", font=("Consolas", 10, "bold"), bg="#E65100", fg="white").pack(side="left", pady=8)

    body = tk.Frame(dialog, padx=15, pady=10)
    body.pack(fill="both", expand=True)

    # Variabel dengan mapping logika
    is_auto_exp = tk.BooleanVar(value=(camera_cfg.get("auto_exposure", 0.25) > 0.5))
    var_exp     = tk.DoubleVar(value=camera_cfg.get("exposure", -5.0))
    is_auto_foc = tk.BooleanVar(value=(camera_cfg.get("autofocus", 0.0) > 0.5))
    var_focus   = tk.IntVar(value=int(camera_cfg.get("focus", 0)))
    is_auto_wb  = tk.BooleanVar(value=(camera_cfg.get("auto_wb", 0.0) > 0.5))

    def apply_changes(*args):
        global cam_settings_changed
        camera_cfg["auto_exposure"] = 0.75 if is_auto_exp.get() else 0.25
        camera_cfg["exposure"]      = var_exp.get()
        camera_cfg["autofocus"]     = 1.0 if is_auto_foc.get() else 0.0
        camera_cfg["focus"]         = float(var_focus.get())
        camera_cfg["auto_wb"]       = 1.0 if is_auto_wb.get() else 0.0
        cam_settings_changed = True

    # --- GRUP EXPOSURE ---
    frm_exp = tk.LabelFrame(body, text="Pencahayaan (Exposure)", font=("Consolas", 9, "bold"), padx=10, pady=5)
    frm_exp.pack(fill="x", pady=5)
    tk.Checkbutton(frm_exp, text="Gunakan Auto Exposure", variable=is_auto_exp, command=apply_changes, font=("Consolas", 9)).pack(anchor="w")
    frm_exp_slider = tk.Frame(frm_exp)
    frm_exp_slider.pack(fill="x", pady=(5,0))
    tk.Label(frm_exp_slider, text="Manual:", font=("Consolas", 9)).pack(side="left")
    tk.Scale(frm_exp_slider, variable=var_exp, from_=-10.0, to=0.0, resolution=1.0, orient="horizontal", command=lambda e: apply_changes()).pack(side="right", fill="x", expand=True, padx=5)

    # --- GRUP FOKUS ---
    frm_foc = tk.LabelFrame(body, text="Ketajaman (Focus)", font=("Consolas", 9, "bold"), padx=10, pady=5)
    frm_foc.pack(fill="x", pady=5)
    tk.Checkbutton(frm_foc, text="Gunakan Auto Focus", variable=is_auto_foc, command=apply_changes, font=("Consolas", 9)).pack(anchor="w")
    frm_foc_slider = tk.Frame(frm_foc)
    frm_foc_slider.pack(fill="x", pady=(5,0))
    tk.Label(frm_foc_slider, text="Manual:", font=("Consolas", 9)).pack(side="left")
    tk.Scale(frm_foc_slider, variable=var_focus, from_=0, to=255, resolution=5, orient="horizontal", command=lambda e: apply_changes()).pack(side="right", fill="x", expand=True, padx=5)

    # --- GRUP WHITE BALANCE ---
    frm_wb = tk.LabelFrame(body, text="Warna (White Balance)", font=("Consolas", 9, "bold"), padx=10, pady=5)
    frm_wb.pack(fill="x", pady=5)
    tk.Checkbutton(frm_wb, text="Gunakan Auto White Balance", variable=is_auto_wb, command=apply_changes, font=("Consolas", 9)).pack(anchor="w")

    def do_save():
        save_config()
        log("[CAM] Pengaturan Kamera disimpan.")
        dialog.destroy()

    frm_btn = tk.Frame(dialog)
    frm_btn.pack(pady=10)
    tk.Button(frm_btn, text="💾  Simpan", command=do_save, bg="#E65100", fg="white", font=("Consolas", 10, "bold"), padx=15).pack()

# =============================================================
# CONNECT / DISCONNECT
# =============================================================
def setup_initial_slaves():
    time.sleep(1.0)
    if not master or not modbus_connected: return
    try:
        belts = get_belt_slots()
        for b in belts:
            sid = b["id"]
            mb_write_reg(sid, REG_BELT_MODE, 3) 
            log(f"[CONFIG] Auto-set Belt C{sid} ke Mode 3")
            time.sleep(0.1)
    except Exception as e:
        log(f"[CONFIG ERR] Gagal set mode awal: {e}")

def init_modbus():
    global master, robots, modbus_connected
    robots = {}
    port   = var_port.get().strip()
    if not port or port == "(tidak ada COM)": return
    try:
        master = minimalmodbus.Instrument(port, SLAVE_IDS[0] if SLAVE_IDS else 1)
        safe_modbus_setup(master)
        modbus_connected = True
        lbl_conn_status.config(text=f"✅  {port} | Slaves {SLAVE_IDS}", fg="#2E7D32")
        log(f"[MODBUS] Connected {port} | Slaves: {SLAVE_IDS}")
        btn_connect.config(text="DISCONNECT", bg="#C62828", command=disconnect_modbus)
        threading.Thread(target=setup_initial_slaves, daemon=True).start()
    except Exception as e:
        master = None; modbus_connected = False
        lbl_conn_status.config(text="❌  Koneksi Gagal", fg="#C62828")
        log(f"[ERR] Modbus: {e}")

def disconnect_modbus():
    global master, modbus_connected
    stop_all()
    modbus_connected = False
    try:
        if master and master.serial.isOpen(): master.serial.close()
    except: pass
    master = None
    lbl_conn_status.config(text="🔌  Tidak Terhubung", fg="#757575")
    log("[MODBUS] Koneksi diputus.")
    btn_connect.config(text="CONNECT", bg="#1A237E", command=init_modbus)

def refresh_com_ports():
    ports = scan_com_ports()
    cb_port["values"] = ports
    if ports: cb_port.current(0)

# =============================================================
# DYNAMIC GUI REFRESH
# =============================================================
cmd_lbls  = {}
prox_lbls = {}

def refresh_live_status_panel():
    for w in frm_stat.winfo_children(): w.destroy()
    cmd_lbls.clear(); prox_lbls.clear()
    for r, slot in enumerate(slave_slots):
        sid   = slot["id"]
        aktif = slot["aktif"]
        fg    = "#212121" if aktif else "#BDBDBD"
        badge = "✅" if aktif else "⛔"
        tk.Label(frm_stat, text=f"{badge} C{sid} {slot['nama']}", font=("Consolas", 9, "bold"), fg=fg, anchor="w").grid(row=r, column=0, sticky="w", pady=2)
        cmd_l  = tk.Label(frm_stat, text="CMD:-",  font=("Consolas", 8), width=8,  anchor="w", fg="#9E9E9E")
        prox_l = tk.Label(frm_stat, text="PROX:○", font=("Consolas", 8), width=8, anchor="w", fg="#9E9E9E")
        cmd_l.grid(row=r, column=1, padx=4)
        prox_l.grid(row=r, column=2)
        if aktif:
            cmd_lbls[sid]  = cmd_l
            prox_lbls[sid] = prox_l

def refresh_target_dropdowns():
    omni_ids = [str(s["id"]) for s in get_omni_slots()]
    combo_target["values"] = omni_ids
    if omni_ids: combo_target.current(0)

def toggle_camera():
    global camera_active, camera_index
    camera_active = not camera_active
    if camera_active:
        camera_index = int(var_cam_idx.get())
        btn_cam_ctrl.config(text="⏹ Stop Kamera", bg="#C62828", fg="white")
    else:
        btn_cam_ctrl.config(text="▶ Start Kamera", bg="#388E3C", fg="white")

# =============================================================
# GUI ROOT & LAYOUT
# =============================================================
root = tk.Tk()
root.title("Auto Sortir  v7.7 (Perfect Routing)")
root.geometry("1100x820")
root.configure(bg="#ECEFF1")

FONT_TITLE  = ("Consolas", 11, "bold")
FONT_NORMAL = ("Consolas", 9)
FONT_SMALL  = ("Consolas", 8)
COLOR_BG    = "#ECEFF1"
COLOR_DARK  = "#1A237E"

frm_left  = tk.Frame(root, bg=COLOR_BG, width=250)
frm_left.pack(side="left", fill="y", padx=(8,4), pady=8)
frm_left.pack_propagate(False)

frm_right = tk.Frame(root, bg=COLOR_BG)
frm_right.pack(side="right", fill="both", expand=True, padx=(4,8), pady=8)

# ── KONEKSI ──────────────────────────────────────────────────
frm_conn = tk.LabelFrame(frm_left, text="Koneksi Modbus", bg=COLOR_BG, font=FONT_NORMAL, padx=6, pady=6)
frm_conn.pack(fill="x", pady=(0,6))
frm_port_row = tk.Frame(frm_conn, bg=COLOR_BG); frm_port_row.pack(fill="x", pady=(0,4))
var_port = tk.StringVar()
ports    = scan_com_ports()
cb_port  = ttk.Combobox(frm_port_row, textvariable=var_port, values=ports, state="readonly", width=12)
cb_port.pack(side="left")
if ports: cb_port.current(0)
tk.Button(frm_port_row, text="⟳ Refresh", command=refresh_com_ports, font=FONT_SMALL).pack(side="right", padx=2)
btn_connect = tk.Button(frm_conn, text="CONNECT", command=init_modbus, bg=COLOR_DARK, fg="white", font=FONT_TITLE, pady=4)
btn_connect.pack(fill="x", pady=(4,4))
lbl_conn_status = tk.Label(frm_conn, text="🔌  Tidak Terhubung", font=FONT_SMALL, fg="#757575", bg=COLOR_BG)
lbl_conn_status.pack()

# ── KAMERA ───────────────────────────────────────────────────
frm_cam_ctrl = tk.LabelFrame(frm_left, text="Kamera", bg=COLOR_BG, font=FONT_NORMAL, padx=6, pady=4)
frm_cam_ctrl.pack(fill="x", pady=(0,6))
frm_cam_row = tk.Frame(frm_cam_ctrl, bg=COLOR_BG); frm_cam_row.pack(fill="x")
tk.Label(frm_cam_row, text="Index:", font=FONT_SMALL, bg=COLOR_BG).pack(side="left")
var_cam_idx = tk.StringVar(value="0")
ttk.Combobox(frm_cam_row, textvariable=var_cam_idx, values=["0","1","2","3"], width=3, state="readonly").pack(side="left", padx=4)
var_det_only = tk.BooleanVar(value=False)
tk.Checkbutton(frm_cam_row, text="Deteksi Saja", variable=var_det_only, command=lambda: globals().update(detection_only_mode=var_det_only.get()), bg=COLOR_BG, font=FONT_SMALL).pack(side="right")
btn_cam_ctrl = tk.Button(frm_cam_ctrl, text="▶ Start Kamera", command=toggle_camera, bg="#388E3C", fg="white", font=FONT_NORMAL, pady=3)
btn_cam_ctrl.pack(fill="x", pady=(4,0))
tk.Button(frm_cam_ctrl, text="📷 Setting Cam", command=open_camera_settings_dialog, font=FONT_SMALL).pack(fill="x", pady=(4,0))

# ── YOLO MODEL ───────────────────────────────────────────────
frm_model = tk.LabelFrame(frm_left, text="Model YOLO", bg=COLOR_BG, font=FONT_NORMAL, padx=6, pady=4)
frm_model.pack(fill="x", pady=(0,6))
model_list    = scan_yolo_models()
var_model_sel = tk.StringVar(value=model_list[0] if model_list else "")
cb_model = ttk.Combobox(frm_model, textvariable=var_model_sel, values=model_list, state="readonly", width=20)
cb_model.pack(fill="x", pady=(0,4))
frm_model_btns = tk.Frame(frm_model, bg=COLOR_BG); frm_model_btns.pack(fill="x")
tk.Button(frm_model_btns, text="🔄 Muat",  command=lambda: reload_yolo_model(var_model_sel.get()), bg=COLOR_DARK, fg="white", font=FONT_SMALL).pack(side="left", padx=(0,4))
tk.Button(frm_model_btns, text="⟳ Scan", command=lambda: cb_model.config(values=scan_yolo_models()), font=FONT_SMALL).pack(side="left")

# ── KONTROL & ROUTING ────────────────────────────────────────
frm_route = tk.LabelFrame(frm_left, text="Kontrol & Routing", bg=COLOR_BG, font=FONT_NORMAL, padx=6, pady=4)
frm_route.pack(fill="x", pady=(0,6))

btn_auto = tk.Button(frm_route, text="▶ START AUTO SORTIR", command=toggle_auto, bg="#388E3C", fg="white", font=FONT_TITLE, pady=6)
btn_auto.pack(fill="x", pady=(0,6))

frm_route_row = tk.Frame(frm_route, bg=COLOR_BG); frm_route_row.pack(fill="x", pady=(0,4))
tk.Label(frm_route_row, text="Target:", font=FONT_SMALL, bg=COLOR_BG).pack(side="left")
omni_ids     = [str(s["id"]) for s in get_omni_slots()]
combo_target = ttk.Combobox(frm_route_row, values=omni_ids, state="readonly", width=5)
combo_target.pack(side="left", padx=4)
if omni_ids: combo_target.current(0)
tk.Label(frm_route_row, text="Arah:", font=FONT_SMALL, bg=COLOR_BG).pack(side="left")
combo_dir = ttk.Combobox(frm_route_row, values=["LURUS","KANAN","KIRI"], state="readonly", width=7)
combo_dir.pack(side="left", padx=4)
combo_dir.current(0)

tk.Button(frm_route, text="🛠  Manual Start", command=start_plan, bg="#455A64", fg="white", font=FONT_NORMAL, pady=2).pack(fill="x", pady=(0,4))
tk.Button(frm_route, text="⏹  Stop All", command=stop_all, bg="#C62828", fg="white", font=FONT_NORMAL, pady=2).pack(fill="x", pady=(0,4))
lbl_plan = tk.Label(frm_route, text="Sistem Siap", fg="#388E3C", font=FONT_SMALL, bg=COLOR_BG, wraplength=220)
lbl_plan.pack()

# ── PENGATURAN SISTEM ─────────────────────────────────────────
frm_popups = tk.LabelFrame(frm_left, text="Pengaturan Sistem", bg=COLOR_BG, font=FONT_NORMAL, padx=6, pady=4)
frm_popups.pack(fill="x", pady=(0,6))

tk.Button(frm_popups, text="⚙  Setup Slave", command=open_slave_config_dialog, bg="#37474F", fg="white", font=FONT_NORMAL, pady=3).pack(fill="x", pady=(0,3))
tk.Button(frm_popups, text="⏱  Timing Delay", command=open_timing_dialog, bg="#00838F", fg="white", font=FONT_NORMAL, pady=3).pack(fill="x", pady=(0,3))
tk.Button(frm_popups, text="⚙  Kalibrasi Motor", command=open_calib_dialog, bg="#E65100", fg="white", font=FONT_NORMAL, pady=3).pack(fill="x", pady=(0,3))
tk.Button(frm_popups, text="🔀  Routing Rules", command=open_routing_dialog, bg="#37474F", fg="white", font=FONT_NORMAL, pady=3).pack(fill="x")

# ── LIVE STATUS ──────────────────────────────────────────────
frm_stat = tk.LabelFrame(frm_left, text="Live Status", bg=COLOR_BG, font=FONT_NORMAL, padx=6, pady=4)
frm_stat.pack(fill="both", expand=True, pady=(6,0))
refresh_live_status_panel()

# ── PANEL KANAN (Kamera & Log) ───────────────────────────────
frm_cam_view = tk.LabelFrame(frm_right, text="Kamera YOLO (Full Area)", bg=COLOR_BG, font=FONT_NORMAL, padx=4, pady=4)
frm_cam_view.pack(fill="x", pady=(0,4))
lbl_video = tk.Label(frm_cam_view, bg="#212121", width=640, height=360); lbl_video.pack()

frm_log_hdr = tk.Frame(frm_right, bg=COLOR_BG); frm_log_hdr.pack(fill="x")
tk.Label(frm_log_hdr, text="Log Sistem", font=FONT_NORMAL, bg=COLOR_BG).pack(side="left")
tk.Button(frm_log_hdr, text="🗑 Clear", command=lambda: (log_box.config(state="normal"), log_box.delete("1.0","end"), log_box.config(state="disabled")), font=FONT_SMALL, bg="#607D8B", fg="white").pack(side="right")
frm_log = tk.Frame(frm_right, bg=COLOR_BG); frm_log.pack(fill="both", expand=True, pady=(2,0))
log_box = tk.Text(frm_log, height=6, font=FONT_SMALL, bg="#263238", fg="#B2EBF2", state="disabled", relief="flat", padx=6, pady=4)
log_box.pack(fill="both", expand=True)

# ── STARTUP ──────────────────────────────────────────────────
_initial_models = scan_yolo_models()
if _initial_models and _initial_models[0] != "(tidak ada .pt)":
    yolo_model_path = _initial_models[0]

threading.Thread(target=camera_capture_thread, daemon=True).start()
threading.Thread(target=yolo_inference_thread, daemon=True).start()
threading.Thread(target=modbus_polling_thread, daemon=True).start()

update_gui_video()
log("[SYSTEM] Auto Sortir v7.7 (Perfect Routing) — Siap.")
root.mainloop()