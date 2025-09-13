import sys
import threading
import time
from dataclasses import dataclass
from typing import Optional

try:
    import serial
    import serial.tools.list_ports
except Exception:
    serial = None

from PyQt6 import QtCore, QtGui, QtWidgets


BAUD_DEFAULT = 115200
STATUS_INTERVAL_MS = 1000
LOG_MAX_LINES = 500
ANGLE_SEND_DEBOUNCE_MS = 300
SPEED_SEND_DEBOUNCE_MS = 300
ANGLE_MIN_DELTA = 0.5
SPEED_MIN_DELTA = 0.5
MOTOR_LABELS = [
    "Gövde",   # M1
    "Omuz",    # M2
    "Dirsek",  # M3
    "Bilek",   # M4
    "El",      # M5
]


@dataclass
class State:
    port: str = ""
    baud: int = BAUD_DEFAULT
    connected: bool = False
    ser: object | None = None


class SerialWorker(QtCore.QObject):
    line_received = QtCore.pyqtSignal(str)
    stopped = QtCore.pyqtSignal()

    def __init__(self, state: State) -> None:
        super().__init__()
        self._state = state
        self._running = False

    def start(self) -> None:
        self._running = True
        threading.Thread(target=self._loop, daemon=True).start()

    def stop(self) -> None:
        self._running = False

    def _loop(self) -> None:
        while self._running and self._state.connected and self._state.ser is not None:
            try:
                line = self._state.ser.readline()
                if line:
                    try:
                        txt = line.decode(errors="ignore").rstrip()
                    except Exception:
                        txt = str(line)
                    self.line_received.emit(txt)
            except Exception:
                time.sleep(0.05)
        self.stopped.emit()


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Açısal Robot Dinamik - PyQt6")
        self.resize(980, 640)

        self.state = State()
        self.worker = SerialWorker(self.state)
        self.worker.line_received.connect(self._append_log)
        self.worker.line_received.connect(self._on_line)

        self._init_ui()
        self._apply_dark()
        self._refresh_ports()
        # Debounce timer'ları
        self._angle_timer: Optional[QtCore.QTimer] = None
        self._speed_timer: Optional[QtCore.QTimer] = None
        self._status_timer: Optional[QtCore.QTimer] = None
        self._last_angle_sent: Optional[float] = None
        self._last_speed_sent: Optional[float] = None
        self._last_status_ms: float = 0.0

    def _init_ui(self) -> None:
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)

        main_layout = QtWidgets.QVBoxLayout(central)

        # Connection panel
        conn_group = QtWidgets.QGroupBox("Bağlantı")
        main_layout.addWidget(conn_group)
        conn_layout = QtWidgets.QGridLayout(conn_group)

        self.cmb_port = QtWidgets.QComboBox()
        self.btn_refresh = QtWidgets.QPushButton("Yenile")
        self.btn_refresh.clicked.connect(self._refresh_ports)
        self.spin_baud = QtWidgets.QSpinBox()
        self.spin_baud.setRange(1200, 1000000)
        self.spin_baud.setValue(BAUD_DEFAULT)
        self.btn_connect = QtWidgets.QPushButton("Bağlan")
        self.btn_connect.clicked.connect(self._toggle_connect)

        conn_layout.addWidget(QtWidgets.QLabel("Port"), 0, 0)
        conn_layout.addWidget(self.cmb_port, 0, 1)
        conn_layout.addWidget(self.btn_refresh, 0, 2)
        conn_layout.addWidget(QtWidgets.QLabel("Baud"), 1, 0)
        conn_layout.addWidget(self.spin_baud, 1, 1)
        conn_layout.addWidget(self.btn_connect, 1, 2)

        # Parameters
        params_group = QtWidgets.QGroupBox("Parametreler")
        main_layout.addWidget(params_group)
        params = QtWidgets.QGridLayout(params_group)

        self.spin_motor = QtWidgets.QSpinBox()
        self.spin_motor.setRange(1, 5)
        self.edit_deg = QtWidgets.QDoubleSpinBox()
        self.edit_deg.setDecimals(1)
        self.edit_deg.setRange(-3600.0, 3600.0)
        self.edit_deg.setSingleStep(1.0)
        self.edit_deg.setValue(15.0)
        self.slider_deg = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal)
        self.slider_deg.setMinimum(-360)
        self.slider_deg.setMaximum(360)
        self.slider_deg.setSingleStep(1)
        self.slider_deg.setPageStep(10)
        self.slider_deg.setValue(15)

        self.edit_dps = QtWidgets.QDoubleSpinBox()
        self.edit_dps.setDecimals(1)
        self.edit_dps.setRange(0.1, 5000.0)
        self.edit_dps.setSingleStep(1.0)
        self.edit_dps.setValue(90.0)
        self.slider_dps = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal)
        self.slider_dps.setMinimum(1)
        self.slider_dps.setMaximum(1000)
        self.slider_dps.setSingleStep(1)
        self.slider_dps.setPageStep(10)
        self.slider_dps.setValue(90)
        self.spin_ramp = QtWidgets.QSpinBox()
        self.spin_ramp.setRange(0, 5000)
        self.spin_ramp.setValue(0)
        self.spin_rep = QtWidgets.QSpinBox()
        self.spin_rep.setRange(1, 200)
        self.spin_rep.setValue(1)
        self.spin_pause = QtWidgets.QSpinBox()
        self.spin_pause.setRange(0, 60000)
        self.spin_pause.setValue(500)

        params.addWidget(QtWidgets.QLabel("Motor"), 0, 0)
        params.addWidget(self.spin_motor, 0, 1)
        params.addWidget(QtWidgets.QLabel("Açı (deg)"), 0, 2)
        params.addWidget(self.edit_deg, 0, 3)
        params.addWidget(self.slider_deg, 0, 4, 1, 2)
        params.addWidget(QtWidgets.QLabel("Hız (deg/s)"), 1, 2)
        params.addWidget(self.edit_dps, 1, 3)
        params.addWidget(self.slider_dps, 1, 4, 1, 2)
        params.addWidget(QtWidgets.QLabel("Rampa (ms)"), 2, 0)
        params.addWidget(self.spin_ramp, 2, 1)
        params.addWidget(QtWidgets.QLabel("Tekrar"), 2, 2)
        params.addWidget(self.spin_rep, 2, 3)
        params.addWidget(QtWidgets.QLabel("Pause (ms)"), 2, 4)
        params.addWidget(self.spin_pause, 2, 5)

        # Buttons
        btn_group = QtWidgets.QGroupBox("Komutlar")
        main_layout.addWidget(btn_group)
        grid = QtWidgets.QGridLayout(btn_group)

        self.btn_m = QtWidgets.QPushButton("Motor Seç (M)")
        # Açı/Hız manuel butonlarını kaldırdık; slider/spinbox otomatik gönderiyor
        self.btn_ramp = QtWidgets.QPushButton("Rampa")
        self.btn_pause = QtWidgets.QPushButton("Pause")
        self.btn_rep = QtWidgets.QPushButton("Tekrar")
        self.btn_g = QtWidgets.QPushButton("G (İleri)")
        self.btn_q = QtWidgets.QPushButton("Q (İleri-Geri)")
        self.btn_move = QtWidgets.QPushButton("MOVE")
        self.btn_test = QtWidgets.QPushButton("TEST")
        self.btn_show = QtWidgets.QPushButton("SHOW")
        self.btn_abort = QtWidgets.QPushButton("ACİL DURDUR")
        self.btn_abort.setStyleSheet("background-color:#b00020;color:white;font-weight:bold;")
        self.btn_ena_on = QtWidgets.QPushButton("ENA ON")
        self.btn_ena_off = QtWidgets.QPushButton("ENA OFF")
        # Net anlaşılır kontrol butonları
        self.btn_start = QtWidgets.QPushButton("BAŞLAT")
        self.btn_stop = QtWidgets.QPushButton("DURDUR")
        self.btn_left = QtWidgets.QPushButton("SOL (−)")
        self.btn_right = QtWidgets.QPushButton("SAĞ (+)")

        # Motor seçim barı (5 toggle buton)
        motor_group = QtWidgets.QGroupBox("Motor Seçimi (çoklu)")
        main_layout.addWidget(motor_group)
        motor_layout = QtWidgets.QHBoxLayout(motor_group)
        self.motor_buttons: list[QtWidgets.QPushButton] = []
        for i in range(1, 6):
            label = MOTOR_LABELS[i-1] if i-1 < len(MOTOR_LABELS) else f"M{i}"
            b = QtWidgets.QPushButton(label)
            b.setCheckable(True)
            b.setMinimumWidth(48)
            b.setToolTip(f"M{i} - {label}")
            b.toggled.connect(lambda checked, m=i: self._on_motor_toggled(m, checked))
            motor_layout.addWidget(b)
            self.motor_buttons.append(b)
        self._update_motor_button_colors()

        grid.addWidget(self.btn_m, 0, 0)
        grid.addWidget(self.btn_ramp, 0, 1)
        grid.addWidget(self.btn_pause, 0, 2)
        grid.addWidget(self.btn_rep, 0, 3)
        grid.addWidget(self.btn_start, 1, 0)
        grid.addWidget(self.btn_stop, 1, 1)
        grid.addWidget(self.btn_left, 1, 2)
        grid.addWidget(self.btn_right, 1, 3)
        grid.addWidget(self.btn_show, 1, 4)
        grid.addWidget(self.btn_abort, 1, 5)
        grid.addWidget(self.btn_g, 2, 0)
        grid.addWidget(self.btn_q, 2, 1)
        grid.addWidget(self.btn_move, 2, 2)
        grid.addWidget(self.btn_test, 2, 3)
        grid.addWidget(self.btn_ena_on, 2, 4)
        grid.addWidget(self.btn_ena_off, 2, 5)

        # Log
        log_group = QtWidgets.QGroupBox("Konsol")
        main_layout.addWidget(log_group, 1)
        vbox = QtWidgets.QVBoxLayout(log_group)
        self.txt_log = QtWidgets.QPlainTextEdit()
        self.txt_log.setReadOnly(True)
        vbox.addWidget(self.txt_log)

        # Connections
        self.btn_m.clicked.connect(self._cmd_select_motor)
        # kaldırıldı: manuel A/V buton bağlantıları
        self.btn_ramp.clicked.connect(self._cmd_set_ramp)
        self.btn_pause.clicked.connect(self._cmd_set_pause)
        self.btn_rep.clicked.connect(self._cmd_set_rep)
        self.btn_g.clicked.connect(self._cmd_G)
        self.btn_q.clicked.connect(self._cmd_Q)
        self.btn_move.clicked.connect(self._cmd_MOVE)
        self.btn_test.clicked.connect(self._cmd_TEST)
        self.btn_show.clicked.connect(self._cmd_SHOW)
        self.btn_abort.clicked.connect(self._cmd_ABORT)
        self.btn_ena_on.clicked.connect(self._cmd_ENA_ON)
        self.btn_ena_off.clicked.connect(self._cmd_ENA_OFF)
        # Yayın (broadcast) davranışı
        self.btn_g.clicked.connect(lambda: self._broadcast_to_selected(self._send_g_for))
        self.btn_q.clicked.connect(lambda: self._broadcast_to_selected(self._send_q_for))
        self.btn_move.clicked.connect(lambda: self._broadcast_to_selected(self._send_move_for))
        self.btn_test.clicked.connect(lambda: self._broadcast_to_selected(self._send_test_for))
        self.btn_start.clicked.connect(lambda: self._broadcast_to_selected(self._send_start_for))
        self.btn_stop.clicked.connect(lambda: self._broadcast_to_selected(self._send_stop_for))
        self.btn_left.clicked.connect(lambda: self._broadcast_to_selected(self._send_left_for))
        self.btn_right.clicked.connect(lambda: self._broadcast_to_selected(self._send_right_for))

        # Sliders <-> SpinBox senkronizasyonu
        self.slider_deg.valueChanged.connect(self._on_slider_deg)
        self.edit_deg.valueChanged.connect(self._on_spin_deg)
        self.slider_deg.sliderReleased.connect(self._schedule_send_angle)
        self.edit_deg.editingFinished.connect(self._schedule_send_angle)
        self.slider_dps.valueChanged.connect(self._on_slider_dps)
        self.edit_dps.valueChanged.connect(self._on_spin_dps)
        self.slider_dps.sliderReleased.connect(self._schedule_send_speed)
        self.edit_dps.editingFinished.connect(self._schedule_send_speed)
        # Sürüklerken sürekli (debounce) gönderme
        self.slider_deg.valueChanged.connect(self._schedule_send_angle)
        self.slider_dps.valueChanged.connect(self._schedule_send_speed)

    def _apply_dark(self) -> None:
        palette = QtGui.QPalette()
        palette.setColor(QtGui.QPalette.ColorRole.Window, QtGui.QColor(45, 45, 45))
        palette.setColor(QtGui.QPalette.ColorRole.WindowText, QtGui.QColor(220, 220, 220))
        palette.setColor(QtGui.QPalette.ColorRole.Base, QtGui.QColor(30, 30, 30))
        palette.setColor(QtGui.QPalette.ColorRole.AlternateBase, QtGui.QColor(45, 45, 45))
        palette.setColor(QtGui.QPalette.ColorRole.Text, QtGui.QColor(220, 220, 220))
        palette.setColor(QtGui.QPalette.ColorRole.Button, QtGui.QColor(60, 60, 60))
        palette.setColor(QtGui.QPalette.ColorRole.ButtonText, QtGui.QColor(220, 220, 220))
        palette.setColor(QtGui.QPalette.ColorRole.Highlight, QtGui.QColor(10, 132, 255))
        palette.setColor(QtGui.QPalette.ColorRole.HighlightedText, QtGui.QColor(0, 0, 0))
        self.setPalette(palette)

        font = self.font()
        font.setPointSize(10)
        self.setFont(font)

    # Serial helpers
    def _refresh_ports(self) -> None:
        self.cmb_port.clear()
        if serial is None:
            self._append_log("pyserial yok. 'pip install pyserial' kurun.")
            return
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.cmb_port.addItems(ports)

    def _toggle_connect(self) -> None:
        if not self.state.connected:
            if serial is None:
                self._append_log("pyserial yok")
                return
            try:
                port = self.cmb_port.currentText()
                baud = int(self.spin_baud.value())
                if not port:
                    QtWidgets.QMessageBox.warning(self, "Uyarı", "Port seçin")
                    return
                self.state.ser = serial.Serial(port=port, baudrate=baud, timeout=0.05)
                time.sleep(0.2)
                self.state.connected = True
                self.btn_connect.setText("Kes")
                self._append_log(f"Bağlandı: {port} @ {baud}")
                self.worker.start()
                # STATUS anketi
                if self._status_timer is None:
                    self._status_timer = QtCore.QTimer(self)
                    self._status_timer.timeout.connect(self._request_status)
                self._status_timer.start(STATUS_INTERVAL_MS)
            except Exception as exc:
                QtWidgets.QMessageBox.critical(self, "Hata", f"Bağlanamadı: {exc}")
        else:
            try:
                if self.state.ser:
                    self.state.ser.close()
                self.state.connected = False
                self.btn_connect.setText("Bağlan")
                self._append_log("Bağlantı kapatıldı")
                self.worker.stop()
                if self._status_timer is not None:
                    self._status_timer.stop()
            except Exception as exc:
                QtWidgets.QMessageBox.critical(self, "Hata", f"Kapatılamadı: {exc}")

    # Commands
    def _send(self, s: str) -> None:
        if not (self.state.connected and self.state.ser):
            self._append_log("Bağlı değil")
            return
        data = (s.strip() + "\n").encode()
        try:
            self.state.ser.write(data)
            self._append_log(f">> {s}")
        except Exception as exc:
            self._append_log(f"Gönderilemedi: {exc}")

    def _cmd_select_motor(self) -> None:
        m = int(self.spin_motor.value())
        self._send(f"M {m}")

    def _cmd_set_angle(self) -> None:
        self._send(f"A {self.edit_deg.value()}")

    def _cmd_set_speed(self) -> None:
        self._send(f"V {self.edit_dps.value()}")

    def _cmd_set_ramp(self) -> None:
        self._send(f"RAMP {self.spin_ramp.value()}")

    def _cmd_set_pause(self) -> None:
        self._send(f"PAUSE {self.spin_pause.value()}")

    def _cmd_set_rep(self) -> None:
        self._send(f"REP {self.spin_rep.value()}")

    def _cmd_G(self) -> None:
        self._cmd_select_motor()
        self._cmd_set_angle()
        self._cmd_set_speed()
        self._cmd_set_ramp()
        self._send("G")

    def _cmd_Q(self) -> None:
        self._cmd_select_motor()
        self._cmd_set_angle()
        self._cmd_set_speed()
        self._cmd_set_ramp()
        self._cmd_set_pause()
        self._cmd_set_rep()
        self._send("Q")

    def _cmd_MOVE(self) -> None:
        m = int(self.spin_motor.value())
        deg = self.edit_deg.value()
        dps = self.edit_dps.value()
        self._cmd_set_ramp()
        self._send(f"MOVE {m} {deg} {dps}")

    def _cmd_TEST(self) -> None:
        m = int(self.spin_motor.value())
        deg = self.edit_deg.value()
        dps = self.edit_dps.value()
        rep = int(self.spin_rep.value())
        pause = int(self.spin_pause.value())
        self._cmd_set_ramp()
        self._send(f"TEST {m} {deg} {dps} {rep} {pause}")

    # Çoklu motor seçimi
    def _on_motor_toggled(self, m: int, checked: bool) -> None:
        self._update_motor_button_colors()

    def _get_selected_motors(self) -> list[int]:
        selected: list[int] = []
        for i, b in enumerate(self.motor_buttons, start=1):
            if b.isChecked():
                selected.append(i)
        # Hiç yoksa spin_motor'daki tekli seçim yedek olsun
        if not selected:
            selected.append(int(self.spin_motor.value()))
        return selected

    def _update_motor_button_colors(self) -> None:
        for b in self.motor_buttons:
            if b.isChecked():
                b.setStyleSheet("background-color:#2e7d32;color:white;font-weight:bold;")
            else:
                b.setStyleSheet("background-color:#8e0000;color:white;")

    # Broadcast yardımcıları
    def _broadcast_to_selected(self, fn) -> None:
        for m in self._get_selected_motors():
            fn(m)

    def _send_g_for(self, m: int) -> None:
        self._send(f"M {m}")
        self._cmd_set_angle()
        self._cmd_set_speed()
        self._cmd_set_ramp()
        self._send("G")

    def _send_q_for(self, m: int) -> None:
        self._send(f"M {m}")
        self._cmd_set_angle()
        self._cmd_set_speed()
        self._cmd_set_ramp()
        self._cmd_set_pause()
        self._cmd_set_rep()
        self._send("Q")

    def _send_move_for(self, m: int) -> None:
        deg = self.edit_deg.value()
        dps = self.edit_dps.value()
        self._cmd_set_ramp()
        self._send(f"MOVE {m} {deg} {dps}")

    def _send_test_for(self, m: int) -> None:
        deg = self.edit_deg.value()
        dps = self.edit_dps.value()
        rep = int(self.spin_rep.value())
        pause = int(self.spin_pause.value())
        self._cmd_set_ramp()
        self._send(f"TEST {m} {deg} {dps} {rep} {pause}")

    # Basit ve net kontroller
    def _send_start_for(self, m: int) -> None:
        # START: ileri (pozitif) açıyla G hareketi
        self._send(f"M {m}")
        self._cmd_set_angle()
        self._cmd_set_speed()
        self._cmd_set_ramp()
        self._send("G")

    def _send_stop_for(self, m: int) -> None:
        self._send(f"STOP {m}")

    def _send_left_for(self, m: int) -> None:
        # SOL: negatif yönde tek adım hareket (MOVE m -|deg| dps)
        deg = -abs(self.edit_deg.value())
        dps = self.edit_dps.value()
        self._cmd_set_ramp()
        self._send(f"MOVE {m} {deg} {dps}")

    def _send_right_for(self, m: int) -> None:
        # SAĞ: pozitif yönde tek adım hareket
        deg = abs(self.edit_deg.value())
        dps = self.edit_dps.value()
        self._cmd_set_ramp()
        self._send(f"MOVE {m} {deg} {dps}")

    def _cmd_SHOW(self) -> None:
        self._send("SHOW")

    def _cmd_ABORT(self) -> None:
        # Anlik olarak abort gonder, ardindan local log yaz
        self._send("ABORT")

    def _cmd_ENA_ON(self) -> None:
        self._send("ENA ON")

    def _cmd_ENA_OFF(self) -> None:
        self._send("ENA OFF")

    def _append_log(self, msg: str) -> None:
        self.txt_log.appendPlainText(msg)
        self._trim_log()

    def _trim_log(self) -> None:
        # Çok satır birikmesini önleyelim
        doc = self.txt_log.document()
        if doc.blockCount() > LOG_MAX_LINES:
            cursor = self.txt_log.textCursor()
            cursor.movePosition(QtGui.QTextCursor.MoveOperation.Start)
            for _ in range(doc.blockCount() - LOG_MAX_LINES):
                cursor.select(QtGui.QTextCursor.SelectionType.LineUnderCursor)
                cursor.removeSelectedText()
                cursor.deleteChar()  # satır sonu
            self.txt_log.setTextCursor(cursor)

    # Status polling and parsing
    def _request_status(self) -> None:
        if not (self.state.connected and self.state.ser):
            return
        now = time.time() * 1000.0
        if now - self._last_status_ms < STATUS_INTERVAL_MS * 0.8:
            return
        self._last_status_ms = now
        self._send("STATUS")

    def _on_line(self, line: str) -> None:
        # Beklenen formatlar:
        # "STATUS M<id> ENA:ON|OFF MOVING:0|1"
        # "MOVING M<id> START|END"
        s = line.strip()
        if s.startswith("STATUS M"):
            try:
                parts = s.split()
                mid = int(parts[1][1:])  # M1
                ena = parts[2].split(":")[1]
                moving = parts[3].split(":")[1]
                self._apply_motor_status(mid, ena == "ON", moving == "1")
            except Exception:
                pass
        elif s.startswith("MOVING M"):
            try:
                parts = s.split()
                mid = int(parts[1][1:])
                is_start = parts[2] == "START"
                self._apply_motor_status(mid, None, is_start)
            except Exception:
                pass

    def _apply_motor_status(self, mid: int, ena_on: Optional[bool], moving: Optional[bool]) -> None:
        # renk: seçili durum yeşil/kırmızı; hareket halinde kenar çizgisi veya ton farkı
        idx = mid - 1
        if 0 <= idx < len(self.motor_buttons):
            b = self.motor_buttons[idx]
            # ENA bilgisi varsa tooltip'e yaz
            tip = b.toolTip() or b.text()
            if ena_on is not None:
                tip = f"M{mid} - {b.text()} | ENA: {'ON' if ena_on else 'OFF'}"
            if moving is not None:
                tip += f" | MOVING: {'1' if moving else '0'}"
            b.setToolTip(tip)
            # Eğer hareket ediyorsa daha parlak bir çerçeve
            base = "#2e7d32" if b.isChecked() else "#8e0000"
            if moving:
                style = f"background-color:{base};color:white;font-weight:bold;border:2px solid #FFC107;"
            else:
                style = f"background-color:{base};color:white;font-weight:bold;"
            b.setStyleSheet(style)

    # Slider sync handlers
    def _on_slider_deg(self, v: int) -> None:
        if abs(self.edit_deg.value() - float(v)) > 0.05:
            self.edit_deg.blockSignals(True)
            self.edit_deg.setValue(float(v))
            self.edit_deg.blockSignals(False)

    def _on_spin_deg(self, v: float) -> None:
        iv = int(round(v))
        if self.slider_deg.value() != iv:
            self.slider_deg.blockSignals(True)
            # Clamp to slider range
            iv = max(self.slider_deg.minimum(), min(self.slider_deg.maximum(), iv))
            self.slider_deg.setValue(iv)
            self.slider_deg.blockSignals(False)

    def _on_slider_dps(self, v: int) -> None:
        if abs(self.edit_dps.value() - float(v)) > 0.05:
            self.edit_dps.blockSignals(True)
            self.edit_dps.setValue(float(v))
            self.edit_dps.blockSignals(False)

    def _on_spin_dps(self, v: float) -> None:
        iv = int(round(v))
        if self.slider_dps.value() != iv:
            self.slider_dps.blockSignals(True)
            iv = max(self.slider_dps.minimum(), min(self.slider_dps.maximum(), iv))
            self.slider_dps.setValue(iv)
            self.slider_dps.blockSignals(False)

    # Auto-send helpers
    def _schedule_send_angle(self) -> None:
        if self._angle_timer is None:
            self._angle_timer = QtCore.QTimer(self)
            self._angle_timer.setSingleShot(True)
            self._angle_timer.timeout.connect(self._maybe_send_angle)
        self._angle_timer.start(ANGLE_SEND_DEBOUNCE_MS)

    def _schedule_send_speed(self) -> None:
        if self._speed_timer is None:
            self._speed_timer = QtCore.QTimer(self)
            self._speed_timer.setSingleShot(True)
            self._speed_timer.timeout.connect(self._maybe_send_speed)
        self._speed_timer.start(SPEED_SEND_DEBOUNCE_MS)

    def _maybe_send_angle(self) -> None:
        if not (self.state.connected and self.state.ser):
            return
        val = float(self.edit_deg.value())
        if self._last_angle_sent is None or abs(val - self._last_angle_sent) >= ANGLE_MIN_DELTA:
            self._cmd_set_angle()
            self._last_angle_sent = val

    def _maybe_send_speed(self) -> None:
        if not (self.state.connected and self.state.ser):
            return
        val = float(self.edit_dps.value())
        if self._last_speed_sent is None or abs(val - self._last_speed_sent) >= SPEED_MIN_DELTA:
            self._cmd_set_speed()
            self._last_speed_sent = val


def main() -> None:
    app = QtWidgets.QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()


