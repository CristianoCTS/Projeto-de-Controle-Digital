from turtle import update
import serial
import time
import sys
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

observed_time = 3600
tempos = []
PWM_values = []
temp_heater = []
temp_ambient = []
raw_heater = []
raw_ambient = []

if len(sys.argv) < 2:
    print("Porta nao especificada.")
    print("Exemplo: python script.py COM3")
    sys.exit(1)

port = sys.argv[1]
rate = 115200
output_file = "Output" + time.strftime("%Y-%m-%d_%H-%M-%S") + ".txt"

try:
    ser = serial.Serial(port, rate, timeout=1)
    time.sleep(2)
    
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=True)
    fig.set_size_inches(fig.get_size_inches()[0] * 2, fig.get_size_inches()[1])
    fig.subplots_adjust(hspace=0.4)

    ln1,     = ax1.plot([], [], 'r-',    label='Heater (°C)')
    ln1_raw, = ax1.plot([], [], 'r-',    label='Heater raw (°C)', alpha=0.3)
    ax1.axhline(y=55, color='orange',  linestyle='--', linewidth=1, label='55°C')
    ax1.axhline(y=65, color='darkred', linestyle='--', linewidth=1, label='65°C')
    ax1.legend(loc='upper left', fontsize='small')
    ax1.set_ylabel('Heater')
    ax1.set_title('Monitoramento Individual')

    ln2,     = ax2.plot([], [], 'b-', label='Ambient (°C)')
    ln2_raw, = ax2.plot([], [], 'b-', label='Ambient raw (°C)', alpha=0.3)
    ax2.legend(loc='upper left', fontsize='small')
    ax2.set_ylabel('Ambient')

    ln3, = ax3.plot([], [], 'g-', label='PWM')
    ax3.legend(loc='upper left', fontsize='small')
    ax3.set_ylabel('Potência')
    ax3.set_xlabel('Tempo (s)')

    def update(frame):
        if ser.in_waiting > 0:
            linha = ser.readline().decode('utf-8').rstrip()
            if ("-|" in linha) and ("| Tempo (s) |" not in linha):
                data = [p.strip() for p in linha[1:].split('|') if p.strip()]
                if len(data) < 6:
                    return ln1, ln1_raw, ln2, ln2_raw, ln3

                t, pwm, th, ta, rh, ra = (float(data[0]), float(data[1]),
                                           float(data[2]), float(data[3]),
                                           float(data[4]), float(data[5]))

                lists = [tempos, PWM_values, temp_heater, temp_ambient, raw_heater, raw_ambient]
                vals  = [t, pwm, th, ta, rh, ra]

                if len(tempos) > observed_time:
                    for lst in lists:
                        lst.pop(0)

                for lst, v in zip(lists, vals):
                    lst.append(v)

                ln1.set_data(tempos, temp_heater)
                ln1_raw.set_data(tempos, raw_heater)
                ln2.set_data(tempos, temp_ambient)
                ln2_raw.set_data(tempos, raw_ambient)
                ln3.set_data(tempos, PWM_values)

                ax3.set_xlim(max(0, tempos[-1] - observed_time), tempos[-1] + 10)

                all_h = temp_heater + raw_heater
                all_a = temp_ambient + raw_ambient
                ax1.set_ylim(min(all_h + [50]) - 5, max(all_h + [70]) + 5)
                ax2.set_ylim(min(all_a) - 5, max(all_a) + 5)
                ax3.set_ylim(min(PWM_values) - 5, max(PWM_values) + 5)

                if linha:
                    arquivo.write(linha + "\n")
                    arquivo.flush()

        return ln1, ln1_raw, ln2, ln2_raw, ln3

    print(f"Lendo {port}")

    arquivo = open(output_file, "a", encoding="utf-8")
    ani = FuncAnimation(fig, update, blit=False, interval=100, cache_frame_data=False)
    plt.show()

except serial.SerialException as e:
    print(f"Erro de conexão: {e}")
finally:
    if 'arquivo' in locals():
        arquivo.close()
    if 'ser' in locals() and ser.is_open:
        ser.close()