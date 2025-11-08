import serial
import threading
import customtkinter as ctk
from collections import deque
import time
import logging
from datetime import datetime

# --- CONFIGURACIÓN ---
SIMULATION_MODE = False      # True: Lee de 'sim_data.txt'. False: Lee del puerto serie.
LOG_TO_FILE = True          # True: Guarda los datos recibidos en modo real.
SIM_FILE_NAME = 'sim_data.txt'
LOG_FILE_NAME = 'telemetry_log.txt'
SERIAL_PORT = 'COM8'
BAUD_RATE = 115200

FLIGHT_STATES = {
    0: "DEBUG", 1: "TEST", 2: "ON PAD", 3: "ASCENDING",
    4: "DROGUE DEPLOY", 5: "MAIN DEPLOY", 6: "LANDED"
}

logging.basicConfig(filename='artemisa_interface.log', level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

# --- CLASE PARA EL WIDGET DE GAUGE ---
class CTkGauge(ctk.CTkFrame):
    def __init__(self, master, title, min_value=0, max_value=100, unit=""):
        super().__init__(master, fg_color="transparent")
        self.min_value = min_value
        self.max_value = max_value
        self.unit = unit
        self.title_label = ctk.CTkLabel(self, text=title, font=ctk.CTkFont(size=14, weight="bold"))
        self.title_label.pack(pady=(5, 0))
        self.value_label = ctk.CTkLabel(self, text=f"--- {unit}", font=ctk.CTkFont(size=20, weight="bold"))
        self.value_label.pack(pady=5)
        self.progress_bar = ctk.CTkProgressBar(self, orientation="horizontal", mode="determinate")
        self.progress_bar.set(0)
        self.progress_bar.pack(pady=(0, 10), padx=10, fill="x")

    def set(self, value):
        try:
            val_num = float(value)
            self.value_label.configure(text=f"{val_num:.1f} {self.unit}")
            progress = (val_num - self.min_value) / (self.max_value - self.min_value)
            progress = max(0, min(1, progress)) 
            self.progress_bar.set(progress)
        except (ValueError, TypeError):
            self.value_label.configure(text=f"--- {self.unit}")
            self.progress_bar.set(0)

# --- CLASE SIMULADORA ---
class FakeSerial:
    def __init__(self, port=None, baudrate=None, timeout=None):
        logging.info(f"Modo simulación activado. Leyendo desde '{SIM_FILE_NAME}'.")
        try:
            with open(SIM_FILE_NAME, 'r') as f:
                self.lines = [line for line in f if not line.strip().startswith('#') and line.strip()]
            if not self.lines: raise FileNotFoundError
        except FileNotFoundError:
            logging.error(f"No se encontró '{SIM_FILE_NAME}'.")
            self.lines = ["STATE:0,ALT:0,SPD:0,ACC_X:0,ROLL:0,PITCH:0,YAW:0,LAT:0,LON:0\n"]
        self.line_index = 0

    def readline(self):
        if not self.lines: time.sleep(1); return b""
        line = self.lines[self.line_index].strip()
        self.line_index = (self.line_index + 1) % len(self.lines)
        time.sleep(0.4)
        return (line + '\n').encode('utf-8')

    def close(self):
        pass

# --- CLASE PRINCIPAL DE LA APLICACIÓN ---
class ArtemisaApp(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.title("Artemisa II - Telemetría en Tiempo Real")
        self.geometry("800x480")
        ctk.set_appearance_mode("dark")

        # Variables de la GUI
        self.flight_state = ctk.StringVar(value="Desconectado")
        self.roll = ctk.StringVar(value="---")
        self.pitch = ctk.StringVar(value="---")
        self.yaw = ctk.StringVar(value="---")
        self.latitude = ctk.StringVar(value="---")
        self.longitude = ctk.StringVar(value="---")
        self.connection_status = ctk.StringVar(value="Iniciando...")

        # Layout de la Interfaz
        self.grid_columnconfigure(0, weight=1)
        self.grid_rowconfigure(1, weight=1)

        title_frame = ctk.CTkFrame(self, fg_color="transparent")
        title_frame.grid(row=0, column=0, sticky="ew", padx=10, pady=10)
        ctk.CTkLabel(title_frame, text="Panel de Control Artemisa II", font=ctk.CTkFont(size=24, weight="bold")).pack()
        
        main_frame = ctk.CTkFrame(self)
        main_frame.grid(row=1, column=0, sticky="nsew", padx=10)
        main_frame.grid_columnconfigure(0, weight=1)
        main_frame.grid_columnconfigure(1, weight=3)

        # Frame para datos de texto
        data_frame = ctk.CTkFrame(main_frame, fg_color="transparent")
        data_frame.grid(row=0, column=0, sticky="nsew", padx=10, pady=10)
        data_frame.grid_columnconfigure(1, weight=1)
        self.create_data_row(data_frame, "Estado de Vuelo:", self.flight_state, 0)
        self.create_data_row(data_frame, "Latitud:", self.latitude, 1)
        self.create_data_row(data_frame, "Longitud:", self.longitude, 2)
        ctk.CTkFrame(data_frame, height=2, fg_color="gray").grid(row=3, column=0, columnspan=2, pady=10, sticky="ew")
        self.create_data_row(data_frame, "Roll:", self.roll, 4)
        self.create_data_row(data_frame, "Pitch:", self.pitch, 5)
        self.create_data_row(data_frame, "Yaw:", self.yaw, 6)
        
        # Frame para los gauges
        gauge_frame = ctk.CTkFrame(main_frame)
        gauge_frame.grid(row=0, column=1, sticky="nsew", padx=10, pady=10)
        gauge_frame.grid_columnconfigure((0, 1, 2), weight=1)

        # Creación de los Gauges
        self.altitude_gauge = CTkGauge(gauge_frame, title="Altitud", min_value=0, max_value=500, unit="m")
        self.altitude_gauge.grid(row=0, column=0, padx=10, pady=10, sticky="ew")
        self.speed_gauge = CTkGauge(gauge_frame, title="Velocidad", min_value=0, max_value=400, unit="km/h")
        self.speed_gauge.grid(row=0, column=1, padx=10, pady=10, sticky="ew")
        self.accel_gauge = CTkGauge(gauge_frame, title="Aceleración X", min_value=-50, max_value=200, unit="m/s²")
        self.accel_gauge.grid(row=0, column=2, padx=10, pady=10, sticky="ew")

        status_bar = ctk.CTkLabel(self, textvariable=self.connection_status, anchor="w")
        status_bar.grid(row=2, column=0, padx=10, pady=(5, 5), sticky="ew")

        # Lógica de comunicación
        self.data_queue = deque(maxlen=1)
        self.running = True
        self.serial_thread = threading.Thread(target=self.read_serial_data, daemon=True)
        self.serial_thread.start()
        self.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.update_gui()

    def create_data_row(self, master, text, textvariable, row):
        ctk.CTkLabel(master, text=text, anchor="w", font=ctk.CTkFont(size=14)).grid(row=row, column=0, padx=10, pady=5, sticky="w")
        ctk.CTkLabel(master, textvariable=textvariable, anchor="w", font=ctk.CTkFont(size=14, weight="bold")).grid(row=row, column=1, padx=10, pady=5, sticky="w")

    def read_serial_data(self):
        logging.info("Hilo de lectura serie iniciado.")
        telemetry_log_file = None
        if LOG_TO_FILE and not SIMULATION_MODE:
            try:
                telemetry_log_file = open(LOG_FILE_NAME, 'a')
                telemetry_log_file.write(f"\n--- INICIO DE SESIÓN DE TELEMETRÍA: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')} ---\n")
            except Exception as e:
                logging.error(f"No se pudo abrir el archivo de log de telemetría: {e}")
        
        while self.running:
            try:
                ser = FakeSerial() if SIMULATION_MODE else serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=2)
                self.connection_status.set(f"Conectado a {'SIMULACIÓN' if SIMULATION_MODE else SERIAL_PORT}")
                while self.running:
                    line_bytes = ser.readline()
                    if line_bytes:
                        line_str = line_bytes.decode('utf-8').strip()
                        self.data_queue.append(line_str)
                        if telemetry_log_file:
                            timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]
                            telemetry_log_file.write(f"{timestamp} -> {line_str}\n")
                            telemetry_log_file.flush()
            except serial.SerialException as e:
                if not SIMULATION_MODE:
                    error_msg = f"Error de conexión en {SERIAL_PORT}. Reintentando..."
                    logging.error(f"{error_msg} - Detalle: {e}")
                    self.connection_status.set(error_msg)
                    self.data_queue.append("CLEAR")
                    time.sleep(2)
            except Exception as e:
                logging.error(f"Error inesperado en el hilo de lectura: {e}", exc_info=True)
                self.connection_status.set(f"Error inesperado. Ver log.")
                time.sleep(2)
        
        if telemetry_log_file:
            telemetry_log_file.close()
            logging.info("Archivo de log de telemetría cerrado.")

    def update_gui(self):
        try:
            line = self.data_queue.popleft()
            if "CLEAR" in line:
                for var in [self.flight_state, self.latitude, self.longitude, self.roll, self.pitch, self.yaw]:
                    var.set("---" if var != self.flight_state else "Desconectado")
                self.altitude_gauge.set(None)
                self.speed_gauge.set(None)
                self.accel_gauge.set(None)
            else:
                data_dict = {key: val for key, val in (part.split(':') for part in line.split(','))}
                
                state_num = int(data_dict.get('STATE', -1))
                self.flight_state.set(FLIGHT_STATES.get(state_num, "Desconocido"))
                self.latitude.set(data_dict.get('LAT', '---'))
                self.longitude.set(data_dict.get('LON', '---'))
                self.roll.set(f"{data_dict.get('ROLL', '---')} °")
                self.pitch.set(f"{data_dict.get('PITCH', '---')} °")
                self.yaw.set(f"{data_dict.get('YAW', '---')} °")
                
                self.altitude_gauge.set(data_dict.get('ALT', 0))
                self.speed_gauge.set(data_dict.get('SPD', 0))
                self.accel_gauge.set(data_dict.get('ACC_X', 0))
        except IndexError:
            pass
        except (ValueError, IndexError) as e:
            logging.warning(f"Línea de datos mal formada: '{line}' - Error: {e}")
        
        self.after(100, self.update_gui)

    def on_closing(self):
        self.running = False
        self.after(200, self.destroy)

if __name__ == "__main__":
    app = ArtemisaApp()
    app.mainloop()