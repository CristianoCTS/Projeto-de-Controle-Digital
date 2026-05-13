from turtle import update
import serial
import time
import sys
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

observed_time = 1800
tempos = []
PWM_values = []
temp_heater = []
temp_ambient = []

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
    fig.subplots_adjust(hspace=0.4)
    ln1, = ax1.plot([], [], 'r-', label='Heater (°C)')
    ax1.axhline(y=55, color='orange', linestyle='--', linewidth=1, label='55°C')
    ax1.axhline(y=65, color='darkred', linestyle='--', linewidth=1, label='65°C')
    ax1.legend(loc='upper left', fontsize='small')
    ax1.set_ylabel('Heater')
    ax1.set_title('Monitoramento Individual')

    ln2, = ax2.plot([], [], 'b-', label='Ambient (°C)')
    ax2.legend(loc='upper left', fontsize='small')
    ax2.set_ylabel('Ambient')

    ln3, = ax3.plot([], [], 'g-', label='PWM')
    ax3.legend(loc='upper left', fontsize='small')
    ax3.set_ylabel('Potência')
    ax3.set_xlabel('Tempo (s)')
    
    def update(frame):
        if (ser.in_waiting > 0):
            linha = ser.readline().decode('utf-8').rstrip()
            if ("-|" in linha) and ("| Tempo (s) |" not in linha):
                data = [p.strip() for p in linha[1:].split('|') if p.strip()]
                if len(tempos) > observed_time:
                    tempos.pop(0)
                    PWM_values.pop(0)
                    temp_heater.pop(0)
                    temp_ambient.pop(0)
                    tempos.append(float(data[0]))
                    PWM_values.append(float(data[1]))
                    temp_heater.append(float(data[2]))
                    temp_ambient.append(float(data[3]))
                else:
                    tempos.append(float(data[0]))
                    PWM_values.append(float(data[1]))
                    temp_heater.append(float(data[2]))
                    temp_ambient.append(float(data[3]))

                ln1.set_data(tempos, temp_heater)
                ln2.set_data(tempos, temp_ambient)
                ln3.set_data(tempos, PWM_values)
                
                ax3.set_xlim(max(0, tempos[-1] - observed_time), tempos[-1] + 10)

                ax1.set_ylim(min(temp_heater + [50]) - 5, max(temp_heater + [70]) + 5)
                ax2.set_ylim(min(temp_ambient) - 5, max(temp_ambient) + 5)
                ax3.set_ylim(min(PWM_values) - 5, max(PWM_values) + 5)
                if linha:
                        arquivo.write(linha + "\n")
                        arquivo.flush()
        return ln1, ln2, ln3

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