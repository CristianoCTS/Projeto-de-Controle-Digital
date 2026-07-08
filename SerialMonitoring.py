import serial
import time
import sys
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.animation import FuncAnimation
from collections import deque

observed_time = 3600

# Dados dos gráficos
tempos        = []
Uobs_values   = []
esin_values   = []
pwm_values    = []   # Uobs - e_sin
Th_values     = []
Ta_values     = []
Target_values = []
W_values      = []   # placeholder → sempre 0

# Últimas linhas brutas para exibição no painel de texto
last_output_lines = deque(maxlen=3)
last_states_lines = deque(maxlen=3)

if len(sys.argv) < 2:
    print("Porta nao especificada.")
    print("Exemplo: python script.py COM3")
    sys.exit(1)

port = sys.argv[1]
rate = 115200
timestamp = time.strftime("%Y-%m-%d_%H-%M-%S")
output_file = "Output" + timestamp + ".txt"
states_file = "States" + timestamp + ".txt"

try:
    ser = serial.Serial(port, rate, timeout=1)
    time.sleep(2)

    fig = plt.figure(figsize=(18, 12))
    fig.patch.set_facecolor('#1e1e2e')

    # ax2 (Temperatura) domina a tela; ax1 (Controle) reduzido pela metade;
    # painel de texto reduzido, dividido em Output | States lado a lado
    gs = gridspec.GridSpec(3, 1, figure=fig, height_ratios=[3, 8, 1], hspace=0.4)

    ax1    = fig.add_subplot(gs[0])
    ax2    = fig.add_subplot(gs[1], sharex=ax1)

    gs_txt = gridspec.GridSpecFromSubplotSpec(1, 2, subplot_spec=gs[2], wspace=0.08)
    ax_txt_out    = fig.add_subplot(gs_txt[0])
    ax_txt_states = fig.add_subplot(gs_txt[1])

    for ax in [ax1, ax2]:
        ax.set_facecolor('#13131f')
        ax.tick_params(colors='#cccccc')
        ax.xaxis.label.set_color('#cccccc')
        ax.yaxis.label.set_color('#cccccc')
        ax.title.set_color('#ffffff')
        for spine in ax.spines.values():
            spine.set_edgecolor('#444466')

    for ax in [ax_txt_out, ax_txt_states]:
        ax.set_facecolor('#0d0d1a')
        ax.axis('off')
        for spine in ax.spines.values():
            spine.set_edgecolor('#444466')

    # ── Gráfico 1: Uobs, e_sin, PWM ───────────────────────────────────────
    ln_uobs, = ax1.plot([], [], color='#4fc3f7', linewidth=1.2, label='Uobs')
    ln_esin, = ax1.plot([], [], color='#66bb6a', linewidth=1.2, label='e_sin')
    ln_pwm,  = ax1.plot([], [], color='#ef5350', linewidth=1.2, label='PWM')
    ax1.axhline(y=0, color='#555577', linestyle='--', linewidth=0.8)
    ax1.legend(loc='upper left', fontsize='small', facecolor='#1e1e2e', labelcolor='white')
    ax1.set_ylabel('Uobs / e_sin / PWM', color='#cccccc')
    ax1.set_title('Controle: Uobs, e_sin e PWM', color='white')

    # ── Gráfico 2: Th, Target, Ta, W ──────────────────────────────────────
    ln_th,     = ax2.plot([], [], color='#ef5350', linewidth=1.4, label='Y (Th)')
    ln_target, = ax2.plot([], [], color='#3DAF1D', linewidth=1.2, linestyle='-', label='R (Target)')
    ln_ta,     = ax2.plot([], [], color='#42a5f5', linewidth=1.2, label='Ta')
    ln_w,      = ax2.plot([], [], color='#FFD600', linewidth=1.2, linestyle='--', label='W')
    ax2.legend(loc='upper left', fontsize='small', facecolor='#1e1e2e', labelcolor='white')
    ax2.set_ylabel('Temperatura (°C)', color='#cccccc')
    ax2.set_xlabel('Tempo (s)', color='#cccccc')
    ax2.set_title('Temperatura: Th, Target, Ta e W', color='white')

    txt_out_obj = ax_txt_out.text(
        0.01, 0.95, '',
        transform=ax_txt_out.transAxes,
        fontsize=6.5,
        verticalalignment='top',
        fontfamily='monospace',
        color='#dddddd',
        wrap=True
    )
    txt_states_obj = ax_txt_states.text(
        0.01, 0.95, '',
        transform=ax_txt_states.transAxes,
        fontsize=6.5,
        verticalalignment='top',
        fontfamily='monospace',
        color='#dddddd',
        wrap=True
    )

    def build_text(lines_deque, header):
        lines = []
        if lines_deque:
            lines.append(header)
            lines.extend(lines_deque)
        return "\n".join(lines)

    def update(frame):
        while ser.in_waiting > 0:
            try:
                linha = ser.readline().decode('utf-8', errors='replace').rstrip()
            except Exception:
                continue

            # ── Linha de dados principais ──────────────────────────────────
            if linha.startswith("-|") and "Tempo (s)" not in linha:
                parts = [p.strip() for p in linha[1:].split('|') if p.strip()]
                # 0=Tempo, 1=PWM, 2=Volts, 3=Th, 4=Th_raw, 5=Ta, 6=Ta_raw, 7=Target, 8=Error
                if len(parts) >= 8:
                    try:
                        t      = float(parts[0])
                        th     = float(parts[3])
                        ta     = float(parts[5])
                        target = float(parts[7])
                        pwm  = float(parts[1])
                        Wperturb  = float(parts[9])

                        if len(tempos) > observed_time:
                            for lst in [tempos, Th_values, Ta_values, Target_values, W_values]:
                                lst.pop(0)

                        tempos.append(t)
                        Th_values.append(th)
                        Ta_values.append(ta)
                        Target_values.append(target)
                        W_values.append(Wperturb)
                        pwm_values.append(pwm)
                        ln_th.set_data(tempos, Th_values)
                        ln_target.set_data(tempos, Target_values)
                        ln_ta.set_data(tempos, Ta_values)
                        ln_w.set_data(tempos, W_values)
                        

                        ax2.set_xlim(max(0, tempos[-1] - observed_time), tempos[-1] + 10)
                        all_temps = Th_values + Ta_values + Target_values + W_values
                        ax2.set_ylim(-20, 90)
                    except (ValueError, IndexError):
                        pass

                last_output_lines.append(linha)
                arquivo_out.write(linha + "\n")
                arquivo_out.flush()

            # ── Linha de estados ───────────────────────────────────────────
            elif linha.startswith("Uobs:"):
                parts_s = {}
                for token in linha.split('|'):
                    token = token.strip()
                    if ':' in token:
                        k, v = token.split(':', 1)
                        parts_s[k.strip()] = v.strip()

                try:
                    uobs = float(parts_s.get('Uobs', 'nan'))
                    esin = float(parts_s.get('e_sin', 'nan'))

                    if tempos:
                        if len(Uobs_values) > observed_time:
                            Uobs_values.pop(0)
                            esin_values.pop(0)
                            pwm_values.pop(0)

                        Uobs_values.append(uobs)
                        esin_values.append(esin)

                        x_uobs = tempos[-len(Uobs_values):]
                        ln_uobs.set_data(x_uobs, Uobs_values)
                        ln_esin.set_data(x_uobs, esin_values)
                        ln_pwm.set_data(x_uobs, pwm_values)

                        all_u = Uobs_values + esin_values + pwm_values
                        ax1.set_ylim(min(all_u) - 50, max(all_u) + 50)

                except (ValueError, KeyError):
                    pass

                last_states_lines.append(linha)
                arquivo_states.write(linha + "\n")
                arquivo_states.flush()

        txt_out_obj.set_text(build_text(last_output_lines, "─── Output ───"))
        txt_states_obj.set_text(build_text(last_states_lines, "─── States ───"))
        return ln_uobs, ln_esin, ln_pwm, ln_th, ln_target, ln_ta, ln_w, txt_out_obj, txt_states_obj

    print(f"Lendo {port} | Output → {output_file} | States → {states_file}")

    arquivo_out    = open(output_file, "a", encoding="utf-8")
    arquivo_states = open(states_file, "a", encoding="utf-8")

    ani = FuncAnimation(fig, update, blit=False, interval=100, cache_frame_data=False)
    plt.show()

except serial.SerialException as e:
    print(f"Erro de conexão: {e}")
finally:
    if 'arquivo_out' in locals():
        arquivo_out.close()
    if 'arquivo_states' in locals():
        arquivo_states.close()
    if 'ser' in locals() and ser.is_open:
        ser.close()